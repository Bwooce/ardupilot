#!/usr/bin/env python3
"""
Serial Port Lock Manager for ESP32 Development

This module provides lock file management for serial port access to prevent
conflicts between different tools (monitor, upload, etc.) accessing the same port.

Uses /var/lock/LCK..ttyUSB* style lock files compatible with standard UNIX conventions.
"""

import os
import sys
import time
import fcntl
import signal
import atexit
from pathlib import Path
from typing import Optional

class SerialPortLock:
    """Manages lock files for serial port access"""

    def __init__(self, port_path: str, tool_name: str = "unknown"):
        """
        Initialize lock manager for a serial port

        Args:
            port_path: Full path to serial port (e.g., /dev/ttyUSB0)
            tool_name: Name of the tool requesting lock (for debugging)
        """
        self.port_path = port_path
        self.tool_name = tool_name
        self.port_name = os.path.basename(port_path)

        # Standard UNIX lock file location
        self.lock_dir = Path("/var/lock")
        if not self.lock_dir.exists():
            # Fallback to /tmp if /var/lock doesn't exist
            self.lock_dir = Path("/tmp")

        self.lock_file = self.lock_dir / f"LCK..{self.port_name}"
        self.lock_fd = None
        self.owns_lock = False

        # Register cleanup on exit
        atexit.register(self.release)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle signals by releasing lock and exiting"""
        self.release()
        sys.exit(0)

    def request_release(self) -> bool:
        """
        Request the current lock holder to release the lock

        Returns:
            True if release was requested, False if no lock exists
        """
        info = self.get_lock_info()
        if info:
            pid, tool = info
            try:
                # Send SIGUSR1 to request graceful release
                os.kill(pid, signal.SIGUSR1)
                print(f"Sent release request to {tool} (PID {pid})")
                return True
            except ProcessLookupError:
                # Process doesn't exist, remove stale lock
                os.unlink(self.lock_file)
                return False
            except Exception as e:
                print(f"Could not request release: {e}")
                return False
        return False

    def acquire(self, timeout: float = 10.0, retry_interval: float = 0.5, auto_request_release: bool = True) -> bool:
        """
        Acquire lock on the serial port

        Args:
            timeout: Maximum time to wait for lock (seconds)
            retry_interval: Time between retry attempts (seconds)
            auto_request_release: Automatically request release from current holder

        Returns:
            True if lock acquired, False if timeout
        """
        start_time = time.time()
        release_requested = False

        while time.time() - start_time < timeout:
            try:
                # Try to create/open lock file
                self.lock_fd = os.open(str(self.lock_file),
                                      os.O_CREAT | os.O_WRONLY | os.O_EXCL)

                # Write PID and tool name to lock file
                lock_content = f"{os.getpid()}\n{self.tool_name}\n"
                os.write(self.lock_fd, lock_content.encode())
                os.fsync(self.lock_fd)

                self.owns_lock = True
                return True

            except FileExistsError:
                # Lock file exists, check if process is still alive
                if self._check_stale_lock():
                    continue  # Stale lock was removed, try again

                # Lock is held by another process
                if auto_request_release and not release_requested:
                    # Request release once
                    if self.request_release():
                        release_requested = True
                        time.sleep(1.0)  # Give holder time to cleanup
                        continue

                # Lock is held by another process
                if timeout > 0:
                    time.sleep(retry_interval)
                else:
                    return False  # Non-blocking mode

            except Exception as e:
                print(f"Error acquiring lock: {e}", file=sys.stderr)
                return False

        # Timeout reached
        return False

    def _check_stale_lock(self) -> bool:
        """
        Check if existing lock is stale (process no longer exists)

        Returns:
            True if lock was stale and removed, False otherwise
        """
        try:
            with open(self.lock_file, 'r') as f:
                lines = f.readlines()
                if lines:
                    pid = int(lines[0].strip())

                    # Check if process exists
                    try:
                        os.kill(pid, 0)  # Signal 0 just checks existence
                        return False  # Process exists, lock is valid
                    except ProcessLookupError:
                        # Process doesn't exist, remove stale lock
                        print(f"Removing stale lock from PID {pid}", file=sys.stderr)
                        os.unlink(self.lock_file)
                        return True

        except (FileNotFoundError, ValueError, PermissionError):
            # Lock file doesn't exist or is corrupted
            try:
                os.unlink(self.lock_file)
                return True
            except FileNotFoundError:
                return True

        return False

    def release(self):
        """Release the lock if we own it"""
        if self.owns_lock and self.lock_fd is not None:
            try:
                os.close(self.lock_fd)
                os.unlink(self.lock_file)
                self.owns_lock = False
                self.lock_fd = None
            except Exception:
                pass  # Ignore errors during cleanup

    def is_locked(self) -> bool:
        """Check if port is currently locked (by any process)"""
        return self.lock_file.exists()

    def get_lock_info(self) -> Optional[tuple[int, str]]:
        """
        Get information about current lock holder

        Returns:
            Tuple of (PID, tool_name) if locked, None otherwise
        """
        try:
            with open(self.lock_file, 'r') as f:
                lines = f.readlines()
                pid = int(lines[0].strip()) if len(lines) > 0 else 0
                tool = lines[1].strip() if len(lines) > 1 else "unknown"
                return (pid, tool)
        except (FileNotFoundError, ValueError):
            return None

    def __enter__(self):
        """Context manager entry"""
        if not self.acquire():
            raise RuntimeError(f"Failed to acquire lock for {self.port_path}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.release()


def wait_for_port(port_path: str, tool_name: str, timeout: float = 30.0) -> SerialPortLock:
    """
    Wait for serial port to become available and acquire lock

    Args:
        port_path: Path to serial port
        tool_name: Name of requesting tool
        timeout: Maximum wait time

    Returns:
        SerialPortLock instance with lock acquired

    Raises:
        RuntimeError: If lock cannot be acquired
    """
    lock = SerialPortLock(port_path, tool_name)

    if lock.is_locked():
        info = lock.get_lock_info()
        if info:
            pid, tool = info
            print(f"Waiting for {port_path} (currently used by {tool} PID {pid})...",
                  file=sys.stderr)

    if not lock.acquire(timeout=timeout):
        raise RuntimeError(f"Timeout waiting for {port_path}")

    return lock


# Example usage for testing
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: serial_port_lock.py <port> [tool_name]")
        sys.exit(1)

    port = sys.argv[1]
    tool = sys.argv[2] if len(sys.argv) > 2 else "test"

    print(f"Acquiring lock for {port}...")
    with SerialPortLock(port, tool) as lock:
        print(f"Lock acquired! Press Ctrl+C to release...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nReleasing lock...")