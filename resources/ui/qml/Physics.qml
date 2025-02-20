import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    visible: true
    width: 800
    height: 600
    color: plt.background0

    ColumnLayout {
        anchors.fill: parent
        spacing: 20
        anchors.margins: 20

        // General Physics Settings
        GroupBox {
            title: "General Physics Settings"
            Layout.fillWidth: true
            background: Rectangle {
                color: plt.background1
                radius: 7
            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }

            ColumnLayout {
                spacing: 15
                anchors.fill: parent

                RowLayout {
                    Label {
                        text: "Type:"
                        color: plt.textColor
                        font.bold: true
                    }
                    ComboBox {
                        id: physicsType
                        model: ["ode", "bullet", "simbody", "dart"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                        onCurrentIndexChanged: updateSolverView()
                    }
                }

                DoubleSpinBox {
                    label: "Max Step Size"
                    decimalPlaces: 5
                    default_value: phy.getter("max_step_size")
                    onSpinBoxvalueChanged: {
                       phy.setter("max_step_size",val)
                    }
                }

                DoubleSpinBox {
                    label: "Real Time Factor"
                    decimalPlaces: 2
                    default_value: phy.getter("real_time_factor")
                    onSpinBoxvalueChanged: {
                        phy.setter("real_time_factor",val)
                    }
                }

                RowLayout
                {
                    Label{
                        text: "Max Contacts"
                        color: plt.textColor

                    }
                    SpinBox{
                        value: phy.getter("max_contacts")
                        onValueModified:
                        {
                            phy.setter("max_contacts",value)
                        }
                    }
                }

                DoubleSpinBox{
                    label:"real_time_update_rate"
                    decimalPlaces: 2
                    default_value: phy.getter("real_time_update_rate")
                    max:10000
                    min:0
                    onSpinBoxvalueChanged: {
                        phy.setter("real_time_update_rate",val)
                    }
                }
            }
        }

        // Solver-Specific Settings
        StackView {
            id: stackView
            Layout.fillWidth: true
            Layout.fillHeight: true
            initialItem: odeSettings
        }
    }

    // Function to Update Solver View Based on ComboBox Selection
    function updateSolverView() {
        switch (physicsType.currentIndex) {
        case 0: // ODE
            stackView.replace(odeSettings);
            break;
        case 1: // Bullet
            stackView.replace(bulletSettings);
            break;
        case 2: // Simbody
            stackView.replace(simbodySettings);
            break;
        case 3: // DART
            stackView.replace(dartSettings);
            break;
        default:
            stackView.replace(odeSettings); // Fallback to ODE
        }
    }

    // ODE Settings Component
    Component {
        id: odeSettings
        GroupBox {
            title: "ODE Settings"
            background: Rectangle {
                color:plt.background1
                radius: 7

            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }
            ColumnLayout {
                spacing: 15
                anchors.fill: parent

                RowLayout {
                    Label {
                        text: "Solver Type:"
                        color:plt.textColor
                    }
                    ComboBox {
                        id: stcb
                        model: ["world", "quick"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                    }
                }

                DoubleSpinBox {
                    label: "Min Step Size"
                    decimalPlaces: 5
                    default_value: phy.getter("odeMin_step_size")
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("odeMin_step_size",val)
                    }
                }

                RowLayout {
                    Label {
                        text: "Island Threads:"
                        color:plt.textColor
                    }
                    SpinBox {
                        value: phy.getter("odeIsland_threads")
                        onValueModified: {
                            phy.setter("odeIsland_threads",value)
                        }
                    }
                }

                RowLayout {
                    Label { text: "Iters:"
                        color:plt.textColor
                    }
                    SpinBox {
                        value: phy.getter("odeIters")
                        onValueModified:
                        {
                            phy.setter("odeIters",value)
                        }
                    }
                }

                DoubleSpinBox {
                    label: "Sor"
                    default_value:  phy.getter("odeSor")
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("odeSor",value)
                    }
                }

                GroupBox {
                    title: "Constraints"
                    Layout.fillWidth: true
                    background: Rectangle {
                        color: plt.background2
                        radius: 7
                    }
                    label: Label {
                           text: parent.title
                           color: plt.textColor // Change this to the desired color
                           font.bold: true // Optional: Customize font properties
                       }
                    ColumnLayout {
                        spacing: 10
                        anchors.fill: parent

                        DoubleSpinBox {
                            label: "CFM"
                            default_value: phy.getter("odeCfm")
                            decimalPlaces: 3
                            onSpinBoxvalueChanged: {
                                phy.setter("odeCfm",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "ERP"
                            default_value: phy.getter("odeErp")
                            decimalPlaces: 3
                            onSpinBoxvalueChanged: {
                                phy.setter("odeErp",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Max Correcting Vel"
                            decimalPlaces: 2
                            max:500
                            default_value: phy.getter("odeContact_max_correcting_vel")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("odeContact_max_correcting_vel",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Surface Layer"
                            decimalPlaces: 4
                            default_value: phy.getter("odeContact_surface_layer")
                            onSpinBoxvalueChanged: {
                                phy.setter("odeContact_surface_layer",value)
                            }
                        }
                    }
                }
            }
        }
    }

    // Bullet Settings Component
    Component {
        id: bulletSettings
        GroupBox {
            title: "Bullet Settings"
            background: Rectangle {
                color: plt.background1
                radius: 7
            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }
            ColumnLayout {
                spacing: 15
                anchors.fill: parent

                RowLayout {
                    Label { text: "Solver Type:" }
                    ComboBox {
                        model: ["sequential_impulse"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                    }
                }

                DoubleSpinBox {
                    label: "Min Step Size"
                    decimalPlaces: 5
                    default_value:phy.getter("bulletMin_step_size")
                    onSpinBoxvalueChanged: {
                        phy.setter("bulletMin_step_size",val)
                    }
                }

                RowLayout {
                    Label {
                        text: "Iters:"
                        color:plt.textColor
                    }
                    SpinBox {
                        value: phy.getter("bulletIters")
                        onValueModified:
                        {
                            phy.setter("bulletIters",value)
                        }
                    }
                }

                DoubleSpinBox {
                    label: "Sor"
                    decimalPlaces: 4
                    default_value:phy.getter("bulletSor")
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("bulletSor",val)

                    }
                }

                GroupBox {
                    title: "Constraints"
                    Layout.fillWidth: true
                    background: Rectangle {
                        color: plt.background2
                        radius: 7
                    }
                    label: Label {
                           text: parent.title
                           color: plt.textColor // Change this to the desired color
                           font.bold: true // Optional: Customize font properties
                       }
                    ColumnLayout {
                        spacing: 10
                        anchors.fill: parent

                        RowLayout {
                            Label { text: "CFM:"
                            color:plt.textColor}
                            SpinBox {
                                value: phy.getter("bulletCfm")
                                onValueModified:
                                {
                                    phy.setter("bulletCfm",value)
                                }
                            }
                        }

                        DoubleSpinBox {
                            label: "ERP"
                            decimalPlaces: 3
                            default_value: phy.getter("bulletErp")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("bulletErp",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Surface Layer"
                            decimalPlaces: 4
                            default_value: phy.getter("bulletContact_surface_layer")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("bulletContact_surface_layer",val)
                            }
                        }

                        RowLayout {
                            Label { text: "Split Impulse:"
                            color:plt.textColor
                            }
                            CheckBox { checked: true }
                        }

                        DoubleSpinBox {
                            label: "Split Impulse Penetration Threshold"
                            default_value: phy.getter("bulletSplit_impulse_penetration_threshold")
                            min: -1
                            decimalPlaces: 3
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("bulletSplit_impulse_penetration_threshold",val)
                            }
                        }
                    }
                }
            }
        }
    }

    // DART Settings Component
    Component {
        id: dartSettings
        GroupBox {
            title: "DART Settings"
            background: Rectangle {
                color: plt.background1
                radius: 7
            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }
            ColumnLayout {
                spacing: 7
                anchors.fill: parent

                RowLayout {
                    Label { text: "Solver Type:" }
                    ComboBox {
                        model: ["dantzig", "pgs"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                    }
                }

                RowLayout {
                    Label { text: "Collision Detector:" }
                    ComboBox {
                        model: ["fcl", "dart", "bullet", "ode"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                    }
                }
            }
        }
    }

    // Simbody Settings Component
    Component {
        id: simbodySettings
        GroupBox {
            title: "Simbody Settings"
            background: Rectangle {
                color: plt.background1
                radius: 7
            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }
            ColumnLayout {
                spacing: 15
                anchors.fill: parent

                DoubleSpinBox {
                    label: "Min Step Size"
                    default_value: phy.getter("simbodyMin_step_size")
                    decimalPlaces: 4
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("simbodyMin_step_size",val)
                    }
                }

                DoubleSpinBox {
                    label: "Accuracy"
                    decimalPlaces: 4
                    default_value: phy.getter("simbodyAccuracy")
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("simbodyAccuracy",val)
                    }
                }

                DoubleSpinBox {
                    label: "Max Transient Velocity"
                    decimalPlaces: 3
                    default_value: phy.getter("simbodyMax_transient_velocity")
                    onSpinBoxvalueChanged:
                    {
                        phy.setter("simbodyMax_transient_velocity",val)
                    }
                }

                GroupBox {
                    title: "Contact Settings"
                    Layout.fillWidth: true
                    background: Rectangle {
                        color: plt.background2
                        radius: 7
                    }
                    label: Label {
                           text: parent.title
                           color: plt.textColor // Change this to the desired color
                           font.bold: true // Optional: Customize font properties
                       }
                    ColumnLayout {
                        spacing: 10
                        anchors.fill: parent

                        DoubleSpinBox {
                            label: "Stiffness"
                            default_value: phy.getter("simbodyStiffness")
                            decimalPlaces: 7
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("simbodyStiffness",val)
                            }
                        }


                            DoubleSpinBox {
                                label: "Dissipation"
                                default_value: phy.getter("simbodyDissipation")
                                min:0
                                max:500
                                decimalPlaces: 4
                                onSpinBoxvalueChanged:
                                {
                                    phy.setter("simbodyDissipation",value)
                                }
                            }


                        DoubleSpinBox {
                            label: "Plastic Coef Restitution"
                            decimalPlaces: 3
                            value: 0.5
                        }

                        DoubleSpinBox {
                            label: "Plastic Impact Velocity"
                            decimalPlaces: 2
                            default_value: phy.getter("simbodyPlastic_coef_restitution")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("simbodyPlastic_coef_restitution",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Static Friction"
                            decimalPlaces: 2
                            default_value: phy.getter("simbodyStatic_friction")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("simbodyStatic_friction",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Dynamic Friction"
                            decimalPlaces: 2
                            default_value: phy.getter("simbodyDynamic_friction")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("simbodyDynamic_friction",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Viscous Friction"
                            decimalPlaces: 2
                            default_value: phy.getter("simbodyViscous_friction")
                            onSpinBoxvalueChanged:
                            {
                                phy.setter("simbodyViscous_friction",val)
                            }
                        }
                    }
                }
            }
        }
    }
}
