"""Generate robot from text description using LLM in background.

This module handles the generation of URDF from text descriptions
and conversion to FreeCAD Cross::Robot objects.
LLM calls are made in a background thread to avoid blocking the UI.
"""

from __future__ import annotations

import re
import threading
from typing import Optional, Callable

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

from .LLM_providers import call_llm_provider
from .robot_from_urdf import robot_from_urdf
from .freecadgui_utils import get_progress_bar, gui_process_events


# System prompt for URDF generation
SYSTEM_PROMPT = (
    "Create a URDF for the robot description provided by the user. "
    "Return only a valid URDF in your response."
)


class RobotGenerationWorker(QtCore.QObject):
    """Worker class for background robot generation.
    
    Signals:
        progress_updated: Emitted when progress changes (value 0-100).
        log_message: Emitted with log message.
        generation_finished: Emitted when generation is complete (success, error_message).
        create_robot_requested: Emitted to request robot creation in main thread (urdf_content).
    """
    
    progress_updated = QtCore.Signal(int)
    log_message = QtCore.Signal(str)
    generation_finished = QtCore.Signal(bool, str)
    create_robot_requested = QtCore.Signal(str)
    
    def __init__(self, user_description: str):
        super().__init__()
        self.user_description = user_description
        self._is_cancelled = False
        self._urdf_content = ""
    
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
        """Internal generation process."""
        if self._is_cancelled:
            return
        
        self.progress_updated.emit(10)
        self.log_message.emit("Starting robot generation from text description...")
        
        # Step 1: Call LLM to generate URDF
        if self._is_cancelled:
            return
        
        self.progress_updated.emit(20)
        self.log_message.emit("Sending request to LLM provider...")
        
        try:
            llm_response = call_llm_provider(
                system_prompt=SYSTEM_PROMPT,
                user_prompt=self.user_description,
                log_callback=self._log_callback
            )
        except Exception as e:
            raise RuntimeError(f"LLM generation failed: {str(e)}")
        
        if self._is_cancelled:
            return
        
        self.progress_updated.emit(60)
        self.log_message.emit("Received response from LLM")
        
        # Step 2: Extract URDF from response
        if self._is_cancelled:
            return
        
        self.progress_updated.emit(70)
        self.log_message.emit("Extracting URDF from response...")
        
        urdf_content = extract_urdf_from_response(llm_response)
        
        if not validate_urdf(urdf_content):
            raise RuntimeError(
                "Failed to extract valid URDF from LLM response. "
                "Please check the response content."
            )
        
        self.log_message.emit(f"URDF extracted successfully ({len(urdf_content)} characters)")
        
        # Step 3: Request robot creation in main thread via signal
        if self._is_cancelled:
            return
        
        self._urdf_content = urdf_content
        self.progress_updated.emit(80)
        self.log_message.emit("Creating Robot from URDF...")
        
        # Emit signal to request robot creation in main thread
        self.create_robot_requested.emit(urdf_content)


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
        self.setModal(True)
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
        self._worker.generation_finished.connect(self._on_generation_finished)
        self._worker.generation_finished.connect(self._thread.quit)
        self._thread.finished.connect(self._on_thread_finished)
        
        # Start thread
        self._thread.start()
    
    def _on_cancel(self):
        """Handle cancel button click."""
        if self._worker:
            self._worker.cancel()
        self.status_label.setText("Cancelling...")
        self.cancel_button.setEnabled(False)
    
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
        if success:
            self.status_label.setText("Generation completed successfully!")
            QtWidgets.QMessageBox.information(
                self,
                "Success",
                "Robot generated successfully!"
            )
        else:
            self.status_label.setText(f"Error: {error_message}")
            QtWidgets.QMessageBox.critical(
                self,
                "Generation Failed",
                f"Failed to generate robot:\n\n{error_message}"
            )
    
    def _on_create_robot_requested(self, urdf_content: str):
        """Handle robot creation request in main thread."""
        try:
            doc = fc.activeDocument()
            if not doc:
                doc = fc.newDocument()
            
            robot = robot_from_urdf(
                doc,
                urdf_content,
                create_without_solids=False,
                remove_solid_splitter=True
            )
            
            if robot:
                self.log_text.appendPlainText(f"Robot '{robot.Label}' created successfully")
                self.progress_bar.setValue(100)
                # Fit view
                fcgui.SendMsgToActiveView('ViewFit')
                self._worker.generation_finished.emit(True, "")
            else:
                self._worker.generation_finished.emit(False, "Robot creation returned None")
        except Exception as e:
            self._worker.generation_finished.emit(False, f"Failed to create robot: {str(e)}")
    
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
            self._thread.wait()
        event.accept()


def show_generation_dialog(parent=None):
    """Show the robot generation dialog.
    
    Args:
        parent: Parent widget.
    """
    dialog = RobotGenerationDialog(parent)
    dialog.exec_()
