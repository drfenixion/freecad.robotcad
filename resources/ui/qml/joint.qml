import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GroupBox {
    title: "Joint Properties"
    Layout.fillWidth: true
    background: Rectangle {
        color: plt.background0
        radius: 7
    }
    label: Label {
        text: parent.title
        color: plt.textColor
        font.bold: true
    }

    ColumnLayout {
        spacing: 10

        // TabBar for navigation
        TabBar {
            id: tabBar
            Layout.fillWidth: true

            TabButton { text: "Basic" }
            TabButton { text: "Axis" }
            TabButton { text: "Physics" }
        }

        // StackLayout to manage tabs
        StackLayout {
            id: stackLayout
            Layout.fillWidth: true
            currentIndex: tabBar.currentIndex

            // Basic Properties Tab
            Item {
                RowLayout {
                    spacing: 10

                    GroupBox {
                        title: "Basic Joint Properties"
                        Layout.fillWidth: true

                        background: Rectangle {
                            color: plt.background1
                            radius: 7
                        }
                        label: Label {
                            text: parent.title
                            color: plt.textColor
                            font.bold: true
                        }

                        ColumnLayout {
                            spacing: 5

                            TextField {
                                id: jointName
                                placeholderText: "Joint Name"
                                Layout.fillWidth: true
                            }

                            ComboBox {
                                id: jointType
                                model: ["continuous", "revolute", "gearbox", "revolute2", "prismatic", "ball", "screw", "universal", "fixed"]
                                Layout.fillWidth: true
                                currentIndex: 0
                            }

                            TextField {
                                id: parentLink
                                placeholderText: "Parent Link"
                                Layout.fillWidth: true
                            }

                            TextField {
                                id: childLink
                                placeholderText: "Child Link"
                                Layout.fillWidth: true
                            }

                            TextField {
                                id: gearboxRatio
                                placeholderText: "Gearbox Ratio"
                                validator: DoubleValidator { bottom: 0.0 }
                                Layout.fillWidth: true
                            }

                            TextField {
                                id: screwThreadPitch
                                placeholderText: "Screw Thread Pitch (m/rev)"
                                validator: DoubleValidator { bottom: 0.0 }
                                Layout.fillWidth: true
                            }
                        }
                    }
                }
            }

            // Axis Properties Tab
            Item {
                RowLayout {
                    spacing: 10

                    GroupBox {
                        title: "Axis Properties"
                        Layout.fillWidth: true

                        background: Rectangle {
                            color: plt.background1
                            radius: 7
                        }
                        label: Label {
                            text: parent.title
                            color: plt.textColor
                            font.bold: true
                        }

                        ColumnLayout {
                            spacing: 5

                            TextField {
                                id: axisXyz
                                placeholderText: "Axis XYZ (x y z)"
                                Layout.fillWidth: true
                            }

                            TextField {
                                id: axisExpressedIn
                                placeholderText: "Expressed In Frame"
                                Layout.fillWidth: true
                            }

                            GroupBox {
                                title: "Dynamics"
                                Layout.fillWidth: true

                                background: Rectangle {
                                    color: plt.background2
                                    radius: 7
                                }
                                label: Label {
                                    text: parent.title
                                    color: plt.textColor
                                    font.bold: true
                                }

                                ColumnLayout {
                                    spacing: 5

                                    TextField {
                                        id: damping
                                        placeholderText: "Damping"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: friction
                                        placeholderText: "Friction"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: springReference
                                        placeholderText: "Spring Reference"
                                        validator: DoubleValidator {}
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: springStiffness
                                        placeholderText: "Spring Stiffness"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }
                                }
                            }

                            GroupBox {
                                title: "Limits"
                                Layout.fillWidth: true

                                background: Rectangle {
                                    color: plt.background2
                                    radius: 7
                                }
                                label: Label {
                                    text: parent.title
                                    color: plt.textColor
                                    font.bold: true
                                }

                                ColumnLayout {
                                    spacing: 5

                                    TextField {
                                        id: lowerLimit
                                        placeholderText: "Lower Limit"
                                        validator: DoubleValidator {}
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: upperLimit
                                        placeholderText: "Upper Limit"
                                        validator: DoubleValidator {}
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: effortLimit
                                        placeholderText: "Effort Limit"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: velocityLimit
                                        placeholderText: "Velocity Limit"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: stiffness
                                        placeholderText: "Stiffness"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: dissipation
                                        placeholderText: "Dissipation"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Physics Properties Tab
            Item {
                RowLayout {
                    spacing: 10

                    GroupBox {
                        title: "Physics Properties"
                        Layout.fillWidth: true

                        background: Rectangle {
                            color: plt.background1
                            radius: 7
                        }
                        label: Label {
                            text: parent.title
                            color: plt.textColor
                            font.bold: true
                        }

                        ColumnLayout {
                            spacing: 5

                            GroupBox {
                                title: "ODE Parameters"
                                Layout.fillWidth: true

                                background: Rectangle {
                                    color: plt.background2
                                    radius: 7
                                }
                                label: Label {
                                    text: parent.title
                                    color: plt.textColor
                                    font.bold: true
                                }

                                ColumnLayout {
                                    spacing: 5

                                    CheckBox {
                                        id: cfmDamping
                                        text: "CFM Damping"
                                    }

                                    CheckBox {
                                        id: implicitSpringDamper
                                        text: "Implicit Spring Damper"
                                    }

                                    TextField {
                                        id: fudgeFactor
                                        placeholderText: "Fudge Factor"
                                        validator: DoubleValidator { bottom: 0.0; top: 1.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: cfm
                                        placeholderText: "CFM"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: erp
                                        placeholderText: "ERP"
                                        validator: DoubleValidator { bottom: 0.0; top: 1.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: bounce
                                        placeholderText: "Bounce"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: maxForce
                                        placeholderText: "Max Force"
                                        validator: DoubleValidator { bottom: 0.0 }
                                        Layout.fillWidth: true
                                    }

                                    TextField {
                                        id: velocity
                                        placeholderText: "Velocity"
                                        validator: DoubleValidator {}
                                        Layout.fillWidth: true
                                    }
                                }
                            }

                            GroupBox {
                                title: "Simbody Parameters"
                                Layout.fillWidth: true

                                background: Rectangle {
                                    color: plt.background2
                                    radius: 7
                                }
                                label: Label {
                                    text: parent.title
                                    color: plt.textColor
                                    font.bold: true
                                }

                                ColumnLayout {
                                    spacing: 5

                                    CheckBox {
                                        id: mustBeLoopJoint
                                        text: "Must Be Loop Joint"
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
