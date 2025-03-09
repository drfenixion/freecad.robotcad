import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    color: plt.background0
    anchors.fill: parent

    // Main layout
    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        // TabBar for navigation
        TabBar {
            id: tabBar
            Layout.fillWidth: true

            TabButton { text: "Link" }
            TabButton { text: "Collision" }
        }

        // StackLayout to manage tabs
        StackLayout {
            id: stackLayout
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex


            Item {
                ScrollView {
                    width: parent.width
                    height: parent.height

                    ColumnLayout {
                        spacing: 10
                        width: parent.width

                        GroupBox {
                            title: "Link Properties"
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

                            GridLayout {
                                rows:4
                                columns: 2
                                LineEdit{
                                    label: "Link Name"
                                    text:prop.getter("Label")
                                    onTextvalChanged: {
                                        prop.setter("Label",txt)
                                        prop.setter("Label2",txt)
                                    }

                                }


                                CheckBox {
                                    id: gravityCheckBox
                                    text: "Affected by Gravity"
                                    checked: true
                                    contentItem: Text
                                    {
                                                     text:gravityCheckBox.text
                                                     color:plt.textColor
                                                     verticalAlignment: Text.AlignVCenter
                                                     leftPadding: gravityCheckBox.indicator.width + gravityCheckBox.spacing
                                                 }
                                }

                                CheckBox {
                                    id: windCheckBox
                                    text: "Affected by Wind"
                                    contentItem: Text
                                    {
                                        text:windCheckBox.text
                                        color:plt.textColor
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: windCheckBox.indicator.width + windCheckBox.spacing

                                    }
                                }

                                CheckBox {
                                    id: selfCollideCheckBox
                                    text: "Self Collide"
                                    contentItem: Text
                                    {
                                        text:selfCollideCheckBox.text
                                        color:plt.textColor
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: selfCollideCheckBox.indicator.width + selfCollideCheckBox.spacing

                                    }
                                }

                                CheckBox {
                                    id: kinematicCheckBox
                                    text: "Kinematic Only"
                                    contentItem: Text
                                    {
                                        text:kinematicCheckBox.text
                                        color:plt.textColor
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: kinematicCheckBox.indicator.width + kinematicCheckBox.spacing

                                    }
                                }

                                CheckBox {
                                    id: baseLinkCheckBox
                                    text: "Must Be Base Link"
                                    contentItem: Text
                                    {
                                        text:baseLinkCheckBox.text
                                        color:plt.textColor
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: baseLinkCheckBox.indicator.width + baseLinkCheckBox.spacing

                                    }
                                }

                                GroupBox {
                                    title: "Velocity Decay"
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
                                        spacing: 12

                                        DoubleSpinBox {
                                            id: linearDecay
                                            label: "Linear Decay"
                                            min:0
                                            Layout.fillWidth: true
                                        }

                                        DoubleSpinBox{
                                            id: angularDecay
                                            label: "Angular Decay"
                                            min:0
                                            Layout.fillWidth: true
                                        }
                                    }
                                }
                            }
                        }

                        // Pose Properties

                        // Inertial Properties
                        InertialUI {
                            Layout.fillWidth: true
                        }
                    }
                }
            }

            // Collision Tab
            CollisionUI
            {

            }



        }
    }
}

