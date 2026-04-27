from __future__ import annotations

from pathlib import Path

import FreeCADGui as fcgui
import FreeCAD as fc

try:
    from PySide import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_gui_utils import WbSettingsGetter
from ..wb_utils import set_workbench_param, get_workbench_param
from .. import wb_globals
from ..LLM_providers import (
    AVAILABLE_PROVIDERS,
    PROVIDER_OPENROUTER,
    PROVIDER_OLLAMA,
    PROVIDER_ROUTERAI,
    get_current_provider,
    set_current_provider,
    get_openrouter_api_key,
    set_openrouter_api_key,
    get_openrouter_model,
    set_openrouter_model,
    get_ollama_model,
    set_ollama_model,
    get_routerai_api_key,
    set_routerai_api_key,
    get_routerai_model,
    set_routerai_model,
    DEFAULT_OPENROUTER_MODEL,
    DEFAULT_OLLAMA_MODEL,
    DEFAULT_ROUTERAI_MODEL,
)


def _warn_if_not_workspace(path: str, gui: bool = True):
    p = Path(path)
    if not (p / 'install/setup.bash').exists():
        warn(f'{path} does not appear to be a valid ROS workspace', gui)


class LLMSettingsDialog(QtWidgets.QDialog):
    """Dialog for configuring LLM providers."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("LLM Provider Settings (OPTIONAL)")
        self.setModal(True)
        self.resize(500, 400)
        self._setup_ui()
        self._load_settings()
    
    def _setup_ui(self):
        """Set up the dialog interface."""
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Provider selection
        provider_group = QtWidgets.QGroupBox("LLM Provider")
        provider_layout = QtWidgets.QFormLayout()
        
        self.provider_combo = QtWidgets.QComboBox()
        self.provider_combo.addItems(AVAILABLE_PROVIDERS)
        provider_layout.addRow("Provider:", self.provider_combo)
        
        provider_group.setLayout(provider_layout)
        layout.addWidget(provider_group)
        
        # Model settings
        models_group = QtWidgets.QGroupBox("Model Settings")
        models_layout = QtWidgets.QFormLayout()
        
        # OpenRouter settings
        self.openrouter_model = QtWidgets.QLineEdit()
        self.openrouter_model.setPlaceholderText(DEFAULT_OPENROUTER_MODEL)
        models_layout.addRow("OpenRouter Model:", self.openrouter_model)
        
        self.openrouter_api_key = QtWidgets.QLineEdit()
        self.openrouter_api_key.setEchoMode(QtWidgets.QLineEdit.Password)
        models_layout.addRow("OpenRouter API Key:", self.openrouter_api_key)
        
        # Ollama settings
        self.ollama_model = QtWidgets.QLineEdit()
        self.ollama_model.setPlaceholderText(DEFAULT_OLLAMA_MODEL)
        models_layout.addRow("Ollama Model:", self.ollama_model)
        
        # RouterAI settings
        self.routerai_model = QtWidgets.QLineEdit()
        self.routerai_model.setPlaceholderText(DEFAULT_ROUTERAI_MODEL)
        models_layout.addRow("RouterAI Model:", self.routerai_model)
        
        self.routerai_api_key = QtWidgets.QLineEdit()
        self.routerai_api_key.setEchoMode(QtWidgets.QLineEdit.Password)
        models_layout.addRow("RouterAI API Key:", self.routerai_api_key)
        
        models_group.setLayout(models_layout)
        layout.addWidget(models_group)
        
        # Connect provider change to update visibility
        self.provider_combo.currentTextChanged.connect(self._on_provider_changed)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch()
        
        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.clicked.connect(self.accept)
        button_layout.addWidget(self.save_button)
        
        self.cancel_button = QtWidgets.QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def _on_provider_changed(self, provider: str):
        """Update UI based on selected provider."""
        # Enable/disable fields based on provider
        is_openrouter = (provider == PROVIDER_OPENROUTER)
        is_ollama = (provider == PROVIDER_OLLAMA)
        is_routerai = (provider == PROVIDER_ROUTERAI)
        
        self.openrouter_model.setEnabled(is_openrouter)
        self.openrouter_api_key.setEnabled(is_openrouter)
        self.ollama_model.setEnabled(is_ollama)
        self.routerai_model.setEnabled(is_routerai)
        self.routerai_api_key.setEnabled(is_routerai)
    
    def _load_settings(self):
        """Load current settings into the dialog."""
        # Set current provider
        current_provider = get_current_provider()
        index = self.provider_combo.findText(current_provider)
        if index >= 0:
            self.provider_combo.setCurrentIndex(index)
        
        # Load API keys
        self.openrouter_api_key.setText(get_openrouter_api_key())
        self.routerai_api_key.setText(get_routerai_api_key())
        
        # Load models
        self.openrouter_model.setText(get_openrouter_model())
        self.ollama_model.setText(get_ollama_model())
        self.routerai_model.setText(get_routerai_model())
        
        # Update UI based on current provider
        self._on_provider_changed(current_provider)
    
    def save_settings(self):
        """Save settings from the dialog."""
        # Save provider
        set_current_provider(self.provider_combo.currentText())
        
        # Save API keys
        set_openrouter_api_key(self.openrouter_api_key.text())
        set_routerai_api_key(self.routerai_api_key.text())
        
        # Save models
        set_openrouter_model(self.openrouter_model.text())
        set_ollama_model(self.ollama_model.text())
        set_routerai_model(self.routerai_model.text())


class WbSettingsGetterWithLLM(WbSettingsGetter):
    """Extended settings getter that includes LLM settings button."""
    
    def _on_form_loaded(self):
        """Hook called after form is loaded but before exec_()."""
        self.form.button_llm_settings.clicked.connect(
            self._on_llm_settings_clicked
        )
    
    def _on_llm_settings_clicked(self):
        """Handle LLM settings button click."""
        llm_dialog = LLMSettingsDialog(self.form)
        if llm_dialog.exec_():
            llm_dialog.save_settings()


class _WbSettingsCommand:
    """The command definition to set up the workbench."""

    def GetResources(self):
        return {
            'Pixmap': 'wb_settings.svg',
            'MenuText': tr('Workbench settings'),
            'Accel': 'W, S',
            'ToolTip': tr('Workbench settings'),
        }

    def IsActive(self):
        return True

    def Activated(self):
        settings_getter = WbSettingsGetterWithLLM(wb_globals.g_ros_workspace)
        if settings_getter.get_settings():
            wb_globals.g_ros_workspace = settings_getter.ros_workspace
            set_workbench_param(wb_globals.PREF_VHACD_PATH, str(settings_getter.vhacd_path))
            set_workbench_param(wb_globals.PREF_OVERCROSS_TOKEN, str(settings_getter.overcross_token))
            set_workbench_param(wb_globals.PREF_ALIGN_Z_AXIS_LCS, settings_getter.align_z_axis_lcs)


fcgui.addCommand('WbSettings', _WbSettingsCommand())
