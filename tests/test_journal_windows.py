"""Persistence fault injection and real Windows sharing locks, temporary files only."""
import ctypes
import json
import os
import subprocess
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

from workflow.state_machine import Journal, JournalWriteError, InstanceLock
from tests.pickdrop_support import PickDropCase


class JournalPersistenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'execution.json'
        self.journal = Journal(self.path)

    def test_transient_permission_retry_same_complete_fsynced_file(self):
        replace = os.replace
        fsync = os.fsync
        seen = []
        def sharing(source, target):
            seen.append(Path(source))
            self.assertEqual(json.loads(Path(source).read_text(encoding='utf-8'))['state'], 'WAIT_TARGET')
            self.assertGreater(sync.call_count, 0)
            if len(seen) < 3:
                raise PermissionError(13, 'sharing violation')
            replace(source, target)
        with patch('workflow.state_machine.os.fsync', wraps=fsync) as sync, \
             patch('workflow.state_machine.os.replace', side_effect=sharing) as call, \
             patch('workflow.state_machine.time.sleep') as sleep:
            self.journal.update(state='WAIT_TARGET')
        self.assertEqual(call.call_count, 3)
        self.assertEqual([c.args[0] for c in sleep.call_args_list], [.02, .05])
        self.assertEqual(len(set(seen)), 1)
        self.assertEqual(seen[0].parent, self.path.parent)
        self.assertNotEqual(seen[0], self.path.with_suffix('.tmp'))
        self.assertEqual(json.loads(self.path.read_text())['state'], 'WAIT_TARGET')

    def test_retry_exhaustion_preserves_committed_file_and_latches_fault(self):
        before = self.path.read_bytes()
        with patch('workflow.state_machine.os.replace', side_effect=PermissionError(13, 'denied')) as replace, \
             patch('workflow.state_machine.time.sleep') as sleep:
            with self.assertRaises(JournalWriteError):
                self.journal.intent('START', 'PICKDROP')
        self.assertEqual(replace.call_count, 6)
        self.assertEqual(sleep.call_count, 5)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.journal.state['state'], 'FAULT')
        self.assertTrue(self.journal.state['outcome_unknown'])
        with patch('workflow.state_machine.os.replace') as replace:
            for method in (lambda: self.journal.update(state='IDLE', outcome_unknown=False), self.journal.save,
                           self.journal.accepted, lambda: self.journal.intent('LOADV', 'payload')):
                with self.assertRaises(JournalWriteError):
                    method()
            replace.assert_not_called()

    def test_no_retry_for_non_sharing_error(self):
        before = self.path.read_bytes()
        with patch('workflow.state_machine.os.replace', side_effect=OSError(28, 'disk full')) as replace, \
             patch('workflow.state_machine.time.sleep') as sleep:
            with self.assertRaises(JournalWriteError):
                self.journal.update(state='WAIT_TARGET')
        replace.assert_called_once()
        sleep.assert_not_called()
        self.assertEqual(self.path.read_bytes(), before)

    def test_fsync_failure_does_not_replace(self):
        before = self.path.read_bytes()
        with patch('workflow.state_machine.os.fsync', side_effect=OSError('fsync failed')), \
             patch('workflow.state_machine.os.replace') as replace:
            with self.assertRaises(JournalWriteError):
                self.journal.update(state='WAIT_TARGET')
            replace.assert_not_called()
        self.assertEqual(self.path.read_bytes(), before)

    def test_legacy_temp_and_execution_lock_never_touched(self):
        legacy = self.path.with_suffix('.tmp')
        ownership = self.path.with_suffix('.lock')
        legacy.write_bytes(b'operator recovery evidence')
        ownership.write_bytes(b'ownership evidence')
        self.journal.update(state='WAIT_TARGET')
        self.assertEqual(legacy.read_bytes(), b'operator recovery evidence')
        self.assertEqual(ownership.read_bytes(), b'ownership evidence')
        self.assertEqual(list(self.path.parent.glob('.execution.json.*.tmp')), [])

    def test_same_path_lock_serializes_direct_save_and_updates(self):
        other = Journal(self.path)
        self.assertIs(self.journal.lock, other.lock)
        paths = set()
        replace = os.replace
        def record(source, target):
            paths.add(Path(source))
            json.loads(Path(source).read_text(encoding='utf-8'))
            replace(source, target)
        def update(index):
            self.journal.update(counter=index)
            self.journal.save()
        with patch('workflow.state_machine.os.replace', side_effect=record):
            with ThreadPoolExecutor(max_workers=4) as pool:
                list(pool.map(update, range(20)))
        self.assertEqual(self.journal.state['revision'], 20)
        self.assertEqual(json.loads(self.path.read_text())['revision'], 20)
        self.assertEqual(len(paths), 40)

    def test_failed_accept_keeps_pending_for_restart_recovery(self):
        self.journal.intent('START', 'PICKDROP')
        with patch('workflow.state_machine.os.replace', side_effect=PermissionError('locked')), \
             patch('workflow.state_machine.time.sleep'):
            with self.assertRaises(JournalWriteError):
                self.journal.accepted()
        self.assertEqual(self.journal.state['pending']['payload'], 'PICKDROP')
        recovered = Journal(self.path)
        self.assertEqual(recovered.state['state'], 'FAULT')
        self.assertTrue(recovered.state['outcome_unknown'])
        self.assertEqual(recovered.state['pending']['payload'], 'PICKDROP')

    def test_startup_does_not_rewrite_unchanged_terminal_journal(self):
        before = self.path.read_bytes()
        with patch('workflow.state_machine.os.replace', side_effect=AssertionError('unexpected startup write')):
            recovered = Journal(self.path)
        self.assertEqual(recovered.state['state'], 'IDLE')
        self.assertEqual(self.path.read_bytes(), before)

    def test_corrupt_or_unreadable_journal_is_not_reset(self):
        self.path.write_bytes(b'{broken evidence')
        with self.assertRaises(JournalWriteError):
            Journal(self.path)
        self.assertEqual(self.path.read_bytes(), b'{broken evidence')
        with patch.object(Path, 'read_text', side_effect=PermissionError('locked')), \
             patch('workflow.state_machine.os.replace') as replace:
            with self.assertRaises(JournalWriteError):
                Journal(self.path)
            replace.assert_not_called()

    def test_missing_primary_with_orphan_temp_never_creates_idle_defaults(self):
        for name in ('execution.tmp','.execution.json.123.orphan.tmp'):
            with tempfile.TemporaryDirectory() as folder:
                primary=Path(folder)/'execution.json'
                recovery=primary.parent/name
                recovery.write_bytes(b'pending command recovery evidence')
                with self.assertRaises(JournalWriteError):
                    Journal(primary)
                self.assertFalse(primary.exists())
                self.assertEqual(recovery.read_bytes(),b'pending command recovery evidence')

    @unittest.skipUnless(os.name == 'nt', 'Windows sharing semantics')
    def test_actual_windows_handle_without_share_delete_recovers_during_backoff(self):
        from ctypes import wintypes as w
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.CreateFileW.argtypes = [w.LPCWSTR,w.DWORD,w.DWORD,ctypes.c_void_p,w.DWORD,w.DWORD,w.HANDLE]
        kernel.CreateFileW.restype = w.HANDLE
        kernel.CloseHandle.argtypes = [w.HANDLE]
        handle = kernel.CreateFileW(str(self.path), 0x80000000, 3, None, 3, 0, None)
        self.assertNotEqual(handle, w.HANDLE(-1).value)
        closed = False
        errors = []
        replace = os.replace
        def observed(source, target):
            try:
                replace(source, target)
            except OSError as exc:
                errors.append(exc.winerror)
                raise
        def release(delay):
            nonlocal closed
            # Called only after the OS refused the first atomic replace.
            self.assertTrue(errors)
            self.assertTrue(kernel.CloseHandle(handle))
            closed = True
        try:
            with patch('workflow.state_machine.os.replace', side_effect=observed), \
                 patch('workflow.state_machine.time.sleep', side_effect=release):
                self.journal.update(state='WAIT_TARGET')
            self.assertIn(errors[0], (5,32,33))
            self.assertEqual(json.loads(self.path.read_text())['state'], 'WAIT_TARGET')
        finally:
            if not closed:
                kernel.CloseHandle(handle)

    def test_controller_owner_lock_excludes_second_process(self):
        with self.subTest('single process owner'):
            lock = InstanceLock(self.path.with_suffix('.lock'))
            try:
                code = "from pathlib import Path; from workflow.state_machine import InstanceLock; InstanceLock(Path(__import__('sys').argv[1]))"
                result = subprocess.run([sys.executable, '-B', '-c', code, str(self.path.with_suffix('.lock'))],
                                        capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
            finally:
                lock.close()

    @unittest.skipUnless(os.name == 'nt', 'Windows sharing semantics')
    def test_actual_persistent_share_lock_preserves_pending_after_restart(self):
        from ctypes import wintypes as w
        self.journal.intent('START','PICKDROP')
        before=self.path.read_bytes()
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.CreateFileW.argtypes=[w.LPCWSTR,w.DWORD,w.DWORD,ctypes.c_void_p,w.DWORD,w.DWORD,w.HANDLE]
        kernel.CreateFileW.restype=w.HANDLE
        kernel.CloseHandle.argtypes=[w.HANDLE]
        handle=kernel.CreateFileW(str(self.path),0x80000000,3,None,3,0,None)
        self.assertNotEqual(handle,w.HANDLE(-1).value)
        try:
            with patch('workflow.state_machine.time.sleep'):
                with self.assertRaises(JournalWriteError):
                    self.journal.accepted()
            self.assertEqual(self.path.read_bytes(),before)
            self.assertTrue(self.journal.state['outcome_unknown'])
        finally:
            kernel.CloseHandle(handle)
        recovered=Journal(self.path)
        self.assertEqual(recovered.state['state'],'FAULT')
        self.assertEqual(recovered.state['pending']['payload'],'PICKDROP')


class JournalCommandTests(PickDropCase):
    def test_retrying_intent_writes_never_retries_robot_start_or_loadv(self):
        replace = os.replace
        calls = 0
        def intermittent(source, target):
            nonlocal calls
            if Path(target) != self.path:
                return replace(source, target)
            calls += 1
            if calls % 2:
                raise PermissionError('temporary scanner handle')
            replace(source, target)
        with patch('workflow.state_machine.os.replace', side_effect=intermittent), \
             patch('workflow.state_machine.time.sleep'):
            result = self.run_cycle()
        self.assertEqual(result['state'], 'COMPLETE', result['error'])
        self.assertEqual(self.starts(), ['PICKDROP'])
        self.assertEqual(sum(name == 'LOADV' for name,data in self.c.robot.client.commands), 2)
        self.assertFalse(any(name == 'MOVL' for name,data in self.c.robot.client.commands))

    def test_persistence_failure_blocks_commands_even_after_disk_recovers(self):
        with patch('workflow.state_machine.os.replace', side_effect=PermissionError('locked')), \
             patch('workflow.state_machine.time.sleep'):
            with self.assertRaises(JournalWriteError):
                self.c._command('START', 'HOME', lambda: self.c.robot.client.execute('START','HOME'))
        self.assertEqual(self.starts(), [])
        with self.assertRaises(RuntimeError):
            self.c.start('journal-fault-request', self.c.journal.state['revision'])
        with self.assertRaises(JournalWriteError):
            self.c._command('LOADV', 'payload', lambda: self.c.robot.client.execute('LOADV','payload'))
        self.assertFalse(any(n in ('START','MOVL','LOADV') for n,d in self.c.robot.client.commands))

    def test_accept_failure_does_not_replay_start(self):
        replace = os.replace
        def fail_after_start(source, target):
            if self.starts():
                raise PermissionError('locked after START reply')
            replace(source, target)
        with patch('workflow.state_machine.os.replace', side_effect=fail_after_start), \
             patch('workflow.state_machine.time.sleep'):
            with self.assertRaises(JournalWriteError):
                self.c._command('START','HOME',lambda:self.c.robot.client.execute('START','HOME'))
        self.assertEqual(self.starts(), ['HOME'])
        recovered = Journal(self.path)
        self.assertEqual(recovered.state['state'], 'FAULT')
        self.assertTrue(recovered.state['outcome_unknown'])
        self.assertEqual(recovered.state['pending']['payload'], 'HOME')
