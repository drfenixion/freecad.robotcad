"""Generate robot from text description using LLM in background.

This module handles the generation of URDF from text descriptions
and conversion to FreeCAD Cross::Robot objects.
LLM calls are made in a background thread to avoid blocking the UI.
"""

from __future__ import annotations

import re
import threading
import time
from typing import Optional, Callable

import FreeCAD as fc
import FreeCADGui as fcgui
from freecad.cross.urdf_loader import UrdfLoader
from freecad.cross.wb_utils import MOD_PATH

try:
    from PySide import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

from ..LLM_providers import call_llm_provider
from ..robot_from_urdf import robot_from_urdf
from ..freecadgui_utils import get_progress_bar, gui_process_events, get_report_view_text


# System prompt for URDF generation
SYSTEM_PROMPT = (
    "Create a URDF for the robot description provided by the user. "
    "Use only clear URDF without any macros. "
    "Dont use not standard tags! "
    "Dont use materials! "
    "Return only a valid URDF in your response."
)


class RobotGenerationWorker(QtCore.QObject):
    """Worker class for background robot generation.
    
    Signals:
        progress_updated: Emitted when progress changes (value 0-100).
        log_message: Emitted with log message.
        generation_finished: Emitted when generation is complete (success, error_message).
        create_robot_requested: Emitted to request robot creation in main thread (urdf_content, attempt).
        robot_creation_result: Received from main thread with result (success, error_message).
    """
    
    progress_updated = QtCore.Signal(int)
    log_message = QtCore.Signal(str)
    generation_finished = QtCore.Signal(bool, str)
    create_robot_requested = QtCore.Signal(str, int)
    robot_creation_result = QtCore.Signal(bool, str)
    
    MAX_RETRIES = 5
    
    def __init__(self, user_description: str):
        super().__init__()
        self.user_description = user_description
        self._is_cancelled = False
        self._urdf_content = ""
        self._result_received = False
        self._result_success = False
        self._result_message = ""
        self._result_event = threading.Event()
    
    def cancel(self):
        """Cancel the generation process."""
        self._is_cancelled = True
    
    def run(self):
        """Run the generation process in background."""
        try:
            self._run_generation()
        except Exception as e:
            self.generation_finished.emit(False, str(e))
    
    def _run_generation(self):
        """Internal generation process with retry logic."""
        if self._is_cancelled:
            self.generation_finished.emit(False, "Cancelled by user")
            return
        
        self.progress_updated.emit(10)
        self.log_message.emit("Starting robot generation from text description...")
        
        last_error = ""
        
        for attempt in range(1, self.MAX_RETRIES + 1):
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            self.log_message.emit(f"Attempt {attempt}/{self.MAX_RETRIES}")
            
            # Step 1: Call LLM to generate URDF
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            self.progress_updated.emit(20)
            self.log_message.emit("Sending request to LLM provider...")
            
            # # DEBUG BREAKPOINT for debugpy - attach debugger and this will pause
            # import debugpy; debugpy.breakpoint()
            
            try:
                llm_response = call_llm_provider(
                    system_prompt=SYSTEM_PROMPT,
                    user_prompt=self.user_description,
                    log_callback=self._log_callback
                )
            except Exception as e:
                last_error = str(e)
                if attempt == self.MAX_RETRIES:
                    raise RuntimeError(f"LLM generation failed: {last_error}")
                else:
                    self.log_message.emit(f"LLM error: {last_error}, retrying...")
                    continue
            
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            self.progress_updated.emit(60)
            self.log_message.emit("Received response from LLM")
            
            # Step 2: Extract URDF from response
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            self.progress_updated.emit(70)
            self.log_message.emit("Extracting URDF from response...")
            urdf_content = extract_urdf_from_response(llm_response)
            
            if not validate_urdf(urdf_content):
                last_error = "Failed to extract valid URDF from LLM response"
                if attempt == self.MAX_RETRIES:
                    raise RuntimeError(last_error)
                else:
                    self.log_message.emit("Invalid URDF extracted, retrying...")
                    continue
            
            self.log_message.emit(f"URDF extracted successfully ({len(urdf_content)} characters)")
            
            # Step 3: Request robot creation in main thread via signal
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            self._urdf_content = urdf_content
            self.progress_updated.emit(80)
            self.log_message.emit("Creating Robot from URDF...")
            
            # Reset result state
            self._result_received = False
            self._result_success = False
            self._result_message = ""
            self._result_event.clear()
            
            # Emit signal to request robot creation in main thread
            self.create_robot_requested.emit(urdf_content, attempt)
            
            # Wait for result from main thread
            self._wait_for_result()
            
            if self._is_cancelled:
                self.generation_finished.emit(False, "Cancelled by user")
                return
            
            if self._result_success:
                # Robot created successfully, no errors in Report view
                self.progress_updated.emit(100)
                self.generation_finished.emit(True, "")
                return
            else:
                # Errors detected in Report view, retry with new LLM call
                last_error = self._result_message
                self.log_message.emit(f"Robot creation had issues: {last_error}")
                if attempt < self.MAX_RETRIES:
                    self.log_message.emit("Closing document and retrying with new LLM call...")
                    continue
                else:
                    self.generation_finished.emit(False, f"Failed after {self.MAX_RETRIES} attempts: {last_error}")
                    return
        
        # Should not reach here, but just in case
        self.generation_finished.emit(False, f"Generation failed after max retries: {last_error}")
    
    def _wait_for_result(self):
        """Wait for the main thread to send the robot creation result."""
        # Use threading.Event for proper thread synchronization
        timeout = 300  # seconds (5 minutes for complex operations)
        
        # Wait for the event to be set or timeout
        signaled = self._result_event.wait(timeout=timeout)
        
        if self._is_cancelled:
            self._result_received = True
            self._result_success = False
            self._result_message = "Cancelled by user"
            return
        
        if not signaled:
            # Timeout occurred
            self._result_received = True
            self._result_success = False
            self._result_message = "Timeout waiting for robot creation result"
            self.log_message.emit("WARNING: Timeout waiting for robot creation result")
            return
    
    def _on_robot_creation_result(self, success: bool, error_message: str):
        """Handle the robot creation result from main thread."""
        self.log_message.emit(f"Received result: success={success}")
        self._result_received = True
        self._result_success = success
        self._result_message = error_message
        self._result_event.set()
    
    def _log_callback(self, message: str):
        """Log callback that emits signal."""
        self.log_message.emit(message)


def extract_urdf_from_response(response: str) -> str:
    """Extract URDF XML from LLM response.
    
    Tries to find URDF content in code blocks or returns the full response
    if it looks like valid URDF.
    
    Args:
        response: Raw LLM response.
    
    Returns:
        Extracted URDF string.
    """
    # Try to extract from markdown code blocks
    xml_block_pattern = r'```(?:xml)?\s*\n(.*?)\n\s*```'
    matches = re.findall(xml_block_pattern, response, re.DOTALL)
    
    if matches:
        # Return the first match that looks like URDF
        for match in matches:
            if '<?xml' in match or '<robot' in match:
                return match.strip()
    
    # If no code block found, check if response itself is URDF
    stripped = response.strip()
    if '<?xml' in stripped or '<robot' in stripped:
        return stripped
    
    # Fallback: return as-is
    return stripped


def validate_urdf(urdf_content: str) -> bool:
    """Basic validation of URDF content.
    
    Args:
        urdf_content: URDF XML string.
    
    Returns:
        True if content looks like valid URDF.
    """
    if not urdf_content:
        return False
    if '<robot' not in urdf_content:
        return False
    if '</robot>' not in urdf_content:
        return False
    return True


class RobotGenerationDialog(QtWidgets.QDialog):
    """Dialog showing progress of robot generation."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Generate Primitive Robot by Text")
        self.setModal(False)  # Modeless dialog - allows interaction with FreeCAD
        self.resize(500, 400)
        self._setup_ui()
    
    def _setup_ui(self):
        """Set up the dialog interface."""
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title_label = QtWidgets.QLabel("<h2>Describe robot to generate</h2>")
        title_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # Text input
        self.text_edit = QtWidgets.QTextEdit()
        self.text_edit.setPlaceholderText("Example: 6-axis robotic arm with gripper, on 4 wheels chassis.")
        self.text_edit.setMinimumHeight(100)
        layout.addWidget(self.text_edit)
        
        # Progress bar
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # Status label
        self.status_label = QtWidgets.QLabel("")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.status_label.setStyleSheet("font-weight: bold; color: #555; font-size: 11px;")
        layout.addWidget(self.status_label)
        
        # Log
        log_group = QtWidgets.QGroupBox("Process Log")
        log_layout = QtWidgets.QVBoxLayout()
        log_layout.setContentsMargins(8, 8, 8, 8)
        
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QtGui.QFont("Consolas", 9))
        self.log_text.setMinimumHeight(150)
        self.log_text.setStyleSheet("background-color: #f5f5f5;")
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch()
        
        self.generate_button = QtWidgets.QPushButton("Generate")
        self.generate_button.setMinimumWidth(100)
        self.generate_button.clicked.connect(self._on_generate)
        button_layout.addWidget(self.generate_button)
        
        self.cancel_button = QtWidgets.QPushButton("Cancel")
        self.cancel_button.setEnabled(False)
        self.cancel_button.setMinimumWidth(100)
        self.cancel_button.clicked.connect(self._on_cancel)
        button_layout.addWidget(self.cancel_button)
        
        self.close_button = QtWidgets.QPushButton("Close")
        self.close_button.setMinimumWidth(100)
        self.close_button.clicked.connect(self.close)
        button_layout.addWidget(self.close_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
        # Store worker reference
        self._worker = None
        self._thread = None
    
    def _on_generate(self):
        """Handle generate button click."""
        description = self.text_edit.toPlainText().strip()
        if not description:
            QtWidgets.QMessageBox.warning(
                self,
                "Empty Description",
                "Please enter a robot description before generating."
            )
            return
        
        # Disable inputs during generation
        self.text_edit.setEnabled(False)
        self.generate_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.log_text.clear()
        
        # Create and start worker thread
        self._worker = RobotGenerationWorker(description)
        self._thread = QtCore.QThread()
        
        self._worker.moveToThread(self._thread)
        
        # Connect signals
        self._thread.started.connect(self._worker.run)
        self._worker.progress_updated.connect(self._on_progress_updated)
        self._worker.log_message.connect(self._on_log_message)
        self._worker.create_robot_requested.connect(self._on_create_robot_requested)
        self._worker.robot_creation_result.connect(self._worker._on_robot_creation_result, QtCore.Qt.DirectConnection)
        self._worker.generation_finished.connect(self._on_generation_finished)
        self._worker.generation_finished.connect(self._thread.quit)
        self._thread.finished.connect(self._on_thread_finished)
        
        # Start thread
        self._thread.start()
    
    def _on_cancel(self):
        """Handle cancel button click."""
        if self._worker and not self._worker._is_cancelled:
            self._worker.cancel()
            self.status_label.setText("Cancelling...")
            self.log_text.appendPlainText("Cancellation requested by user...")
            self.cancel_button.setEnabled(False)
            self.generate_button.setEnabled(False)
    
    def _on_progress_updated(self, value: int):
        """Handle progress update."""
        self.progress_bar.setValue(value)
        gui_process_events()
    
    def _on_log_message(self, message: str):
        """Handle log message."""
        self.log_text.appendPlainText(message)
        # Auto-scroll to bottom
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def _on_generation_finished(self, success: bool, error_message: str):
        """Handle generation completion."""
        # Re-enable UI elements
        self.text_edit.setEnabled(True)
        self.generate_button.setEnabled(True)
        self.cancel_button.setEnabled(False)
        self.close_button.setEnabled(True)
        
        if success:
            self.status_label.setText("Generation completed successfully!")
            self.log_text.appendPlainText("Generation completed successfully!")
        elif error_message == "Cancelled by user":
            self.status_label.setText("Canceled")
            self.log_text.appendPlainText(error_message)
        else:
            self.status_label.setText(f"Error: {error_message}")
            self.log_text.appendPlainText(f"Error: {error_message}")
            QtWidgets.QMessageBox.critical(
                self,
                "Generation Failed",
                f"Failed to generate robot:\n\n{error_message}"
            )
    
    def _on_create_robot_requested(self, urdf_content: str, attempt: int):
        """Handle robot creation request in main thread with Report view comparison."""
        try:
            doc = fc.newDocument()
            fc.setActiveDocument(doc.Name)
            QtWidgets.QApplication.processEvents()
            
            # Capture Report view before robot_from_urdf
            report_before = get_report_view_text()
            
            urdf_robot = UrdfLoader.load_from_xacro_string(urdf_content)
            robot = robot_from_urdf(
                doc,
                urdf_robot,
                create_without_solids=False,
                remove_solid_splitter=True
            )
            
            QtWidgets.QApplication.processEvents()
            
            # Capture Report view after robot_from_urdf
            report_after = get_report_view_text()
            
            # Check if there's a difference (new messages appeared)
            report_has_errors = report_before != report_after
            
            if robot and not report_has_errors:
                self.log_text.appendPlainText(f"Robot '{robot.Label}' created successfully")
                
                # Set ABS-Generic material to the robot
                import os
                from pathlib import Path
                abs_material_path = MOD_PATH / 'resources' / 'materials' / 'Standard' / 'Thermoplast' / 'ABS-Generic.FCMat'
                abs_material_path = str(abs_material_path)
                robot.MaterialCardName = "ABS-Generic"
                robot.MaterialCardPath = abs_material_path
                robot.MaterialDensity = "1060 kg/m^3"
                self.log_text.appendPlainText("Set ABS-Generic material to robot")
                
                self.progress_bar.setValue(90)
                doc.recompute()
                
                # Select the robot and call the existing command
                fcgui.Selection.clearSelection()
                fcgui.Selection.addSelection(robot)
                
                # Send success result to worker BEFORE running potentially blocking command
                self._worker.robot_creation_result.emit(True, "")
                QtWidgets.QApplication.processEvents()
                
                # Calculate mass and inertia using existing command (may take time)
                self.log_text.appendPlainText("Calculating mass and inertia...")
                try:
                    fcgui.runCommand('CalculateMassAndInertia')
                except Exception as e:
                    self.log_text.appendPlainText(f"Warning: CalculateMassAndInertia failed: {e}")
                
                self.progress_bar.setValue(100)
                # Fit view
                fcgui.SendMsgToActiveView('ViewFit')
            else:
                # Either robot is None or there were errors in Report view
                if report_has_errors:
                    # Get the new messages
                    new_messages = report_after[len(report_before):].strip()
                    error_msg = f"Errors during robot creation (attempt {attempt}):\n{new_messages}"
                    # self.log_text.appendPlainText(f"Report view errors detected:\n{new_messages}")
                else:
                    error_msg = "Robot creation returned None"
                    self.log_text.appendPlainText(error_msg)
                
                QtWidgets.QApplication.processEvents()
                
                # Close the document since it had errors
                try:
                    fc.closeDocument(doc.Name)
                    self.log_text.appendPlainText(f"Closed document '{doc.Name}' due to errors")
                    QtWidgets.QApplication.processEvents()
                except Exception:
                    pass
                
                # Send failure result to worker for retry
                self._worker.robot_creation_result.emit(False, error_msg)
        except Exception as e:
            # Close document if it was created
            try:
                fc.closeDocument(doc.Name)
            except Exception:
                pass
            
            error_msg = f"Failed to create robot: {str(e)}"
            self.log_text.appendPlainText(error_msg)
            self._worker.robot_creation_result.emit(False, error_msg)
    
    def _on_thread_finished(self):
        """Handle thread completion."""
        self.text_edit.setEnabled(True)
        self.generate_button.setEnabled(True)
        self.cancel_button.setEnabled(False)
    
    def closeEvent(self, event):
        """Handle dialog close."""
        if self._thread and self._thread.isRunning():
            if self._worker:
                self._worker.cancel()
            # Don't block UI thread - just accept the close event
            # The thread will be cleaned up when it finishes
            self._thread.finished.connect(self._thread.deleteLater)
        event.accept()


def show_generation_dialog(parent=None):
    """Show the robot generation dialog (modeless).
    
    Args:
        parent: Parent widget.
    """
    dialog = RobotGenerationDialog(parent)
    dialog.show()
