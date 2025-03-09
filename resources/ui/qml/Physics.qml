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
                        currentIndex: physicsType.model.indexOf(prop.getter("dynamicsengine"))
                        Layout.preferredWidth: 200
                        onCurrentIndexChanged:
                        {
                            var currentIt = physicsType.model[physicsType.currentIndex]
                            updateSolverView()
                            prop.setter("dynamicsengine",currentIt)
                        }
                    }
                }

                DoubleSpinBox {
                    label: "Max Step Size"
                    decimalPlaces: 5
                    default_value: prop.getter("max_step_size")
                    onSpinBoxvalueChanged: {
                       prop.setter("max_step_size",val)
                    }
                }

                DoubleSpinBox {
                    label: "Real Time Factor"
                    decimalPlaces: 2
                    default_value: prop.getter("real_time_factor")
                    onSpinBoxvalueChanged: {
                        prop.setter("real_time_factor",val)
                    }
                }

                CustomSpinBox
                {
                    label:"Max Contacts"
                    objectpropertyName:"max_contacts"
                }

                DoubleSpinBox{
                    label:"real_time_update_rate"
                    decimalPlaces: 2
                    default_value: prop.getter("real_time_update_rate")
                    max:10000
                    min:0
                    onSpinBoxvalueChanged: {
                        prop.setter("real_time_update_rate",val)
                    }
                }
            }
        }

        // Solver-Specific Settings
        StackView {
            id: stackView
            Layout.fillWidth: true
            Layout.fillHeight: true
            initialItem: componentProvider(physicsType.currentIndex)
        }
    }
    function componentProvider(index) {
            switch (index) {
                case 0: return odeSettings
                case 1: return bulletSettings
                case 2: return simbodySettings
                case 3: return dartSettings
                default: return odeSettings // Fallback
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

               GridLayout{
                   columns:2
                    columnSpacing: 10
                   RowLayout {
                       Label {
                           text: "Solver Type:"
                           color:plt.textColor
                       }
                       ComboBox {
                           id: odesolverType
                           model: ["world", "quick"]
                           currentIndex: odesolverType.model.indexOf(prop.getter("odeType"))
                           Layout.preferredWidth: 200
                           onCurrentIndexChanged:
                           {
                               var currentItem = odesolverType.model[odesolverType.currentIndex]
                               prop.setter("odeType",currentItem)
                           }
                       }
                   }

                   DoubleSpinBox {
                       label: "Min Step Size"
                       decimalPlaces: 5
                       default_value: prop.getter("odeMin_step_size")
                       onSpinBoxvalueChanged:
                       {
                           prop.setter("odeMin_step_size",val)
                       }
                   }
                    CustomSpinBox
                    {
                        label:"Island Threads: "
                        objectpropertyName: "odeIsland_threads"
                    }
                    CustomSpinBox
                    {
                        label:"Iters:"
                        objectpropertyName: "odeIters"
                    }


                   DoubleSpinBox {
                       label: "Sor"
                       default_value:  prop.getter("odeSor")
                       onSpinBoxvalueChanged:
                       {
                           prop.setter("odeSor",value)
                       }
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
                            default_value: prop.getter("odeCfm")
                            decimalPlaces: 3
                            onSpinBoxvalueChanged: {
                                prop.setter("odeCfm",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "ERP"
                            default_value: prop.getter("odeErp")
                            decimalPlaces: 3
                            onSpinBoxvalueChanged: {
                                prop.setter("odeErp",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Max Correcting Vel"
                            decimalPlaces: 2
                            max:500
                            default_value: prop.getter("odeContact_max_correcting_vel")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("odeContact_max_correcting_vel",value)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Surface Layer"
                            decimalPlaces: 4
                            default_value: prop.getter("odeContact_surface_layer")
                            onSpinBoxvalueChanged: {
                                prop.setter("odeContact_surface_layer",value)
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
                    Label {
                        text: "Solver Type:"
                        color:plt.textColor
                    }
                    ComboBox {
                        model: ["sequential_impulse"]
                        currentIndex: 0
                        Layout.preferredWidth: 200
                    }
                }

                DoubleSpinBox {
                    label: "Min Step Size"
                    decimalPlaces: 5
                    default_value:prop.getter("bulletMin_step_size")
                    onSpinBoxvalueChanged: {
                        prop.setter("bulletMin_step_size",val)
                    }
                }
                CustomSpinBox{
                    label:"Iters:"
                    objectpropertyName: "bulletIters"
                }


                DoubleSpinBox {
                    label: "Sor"
                    decimalPlaces: 4
                    default_value:prop.getter("bulletSor")
                    onSpinBoxvalueChanged:
                    {
                        prop.setter("bulletSor",val)

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

                        CustomSpinBox
                        {
                            label:"CFM:"
                            objectpropertyName: "bulletCfm"
                        }


                        DoubleSpinBox {
                            label: "ERP"
                            decimalPlaces: 3
                            default_value: prop.getter("bulletErp")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("bulletErp",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Contact Surface Layer"
                            decimalPlaces: 4
                            default_value: prop.getter("bulletContact_surface_layer")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("bulletContact_surface_layer",val)
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
                            default_value: prop.getter("bulletSplit_impulse_penetration_threshold")
                            min: -1
                            decimalPlaces: 3
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("bulletSplit_impulse_penetration_threshold",val)
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
                        id:dartSolver_type
                        model: ["dantzig", "pgs"]
                        currentIndex: dartSolver_type.model.indexOf(prop.getter("dartSolver_type"))
                        Layout.preferredWidth: 200
                        onCurrentIndexChanged:
                        {
                            var currentItem = dartSolver_type.model[dartSolver_type.currentIndex]
                            prop.setter("dartSolver_type",currentItem)
                        }
                    }
                }

                RowLayout {
                    Label { text: "Collision Detector:" }
                    ComboBox {
                        id:dartCollision_detector
                        model: ["fcl", "dart", "bullet", "ode"]
                        currentIndex: dartCollision_detector.model.indexOf(prop.getter("dartCollision_detector"))
                        Layout.preferredWidth: 200
                        onCurrentIndexChanged:
                        {
                            var currentItem = dartCollision_detector.model[dartCollision_detector.currentIndex]
                            prop.setter("dartCollision_detector",currentItem)
                        }

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
                    default_value: prop.getter("simbodyMin_step_size")
                    decimalPlaces: 4
                    onSpinBoxvalueChanged:
                    {
                        prop.setter("simbodyMin_step_size",val)
                    }
                }

                DoubleSpinBox {
                    label: "Accuracy"
                    decimalPlaces: 4
                    default_value: prop.getter("simbodyAccuracy")
                    onSpinBoxvalueChanged:
                    {
                        prop.setter("simbodyAccuracy",val)
                    }
                }

                DoubleSpinBox {
                    label: "Max Transient Velocity"
                    decimalPlaces: 3
                    default_value: prop.getter("simbodyMax_transient_velocity")
                    onSpinBoxvalueChanged:
                    {
                        prop.setter("simbodyMax_transient_velocity",val)
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
                            default_value: prop.getter("simbodyStiffness")
                            decimalPlaces: 7
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("simbodyStiffness",val)
                            }
                        }


                            DoubleSpinBox {
                                label: "Dissipation"
                                default_value: prop.getter("simbodyDissipation")
                                min:0
                                max:500
                                decimalPlaces: 4
                                onSpinBoxvalueChanged:
                                {
                                    prop.setter("simbodyDissipation",value)
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
                            default_value: prop.getter("simbodyPlastic_coef_restitution")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("simbodyPlastic_coef_restitution",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Static Friction"
                            decimalPlaces: 2
                            default_value: prop.getter("simbodyStatic_friction")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("simbodyStatic_friction",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Dynamic Friction"
                            decimalPlaces: 2
                            default_value: prop.getter("simbodyDynamic_friction")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("simbodyDynamic_friction",val)
                            }
                        }

                        DoubleSpinBox {
                            label: "Viscous Friction"
                            decimalPlaces: 2
                            default_value: prop.getter("simbodyViscous_friction")
                            onSpinBoxvalueChanged:
                            {
                                prop.setter("simbodyViscous_friction",val)
                            }
                        }
                    }
                }
            }
        }
    }
}
