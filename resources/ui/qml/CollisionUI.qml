import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GroupBox {
    title: "Collision Properties"
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
    clip: true

    ScrollView{
        width: parent.width
        height: parent.height

        ColumnLayout
        {
            spacing: 7
            RowLayout {
                spacing: 10

                GroupBox {
                    title: "Basic Collision Properties"
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
                        LineEdit{
                            label: "Collision Name"
                            text:prop.getter("name")
                            onTextvalChanged:
                            {
                                prop.setter("name",txt)

                            }
                        }

                        DoubleSpinBox{
                            label: "Laser Retro"
                            min:0
                            Layout.fillWidth: true
                        }


                    }
                }

                GroupBox {
                    title: "Bounce Properties"
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
                        DoubleSpinBox
                        {
                            label:"Restitution coefficient"
                            min:0
                            max:1

                        }

                        DoubleSpinBox{
                            label:"Bounce Threshold"
                            min:0
                            max:1

                        }

                    }
                }
            }
            GroupBox {
                title: "Friction Properties"
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

                RowLayout {
                    spacing: 12

                    // Torsional Friction
                    GroupBox {
                        title: "Torsional Friction"
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
                            DoubleSpinBox
                            {
                                id:torsionalcoeff
                                label:"Torsional Coefficient"
                                min:0

                            }


                            CheckBox {
                                id: usePatchRadius
                                contentItem: Text
                                {
                                    text:usePatchRadius.text
                                    color:plt.textColor
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: usePatchRadius.indicator.width + usePatchRadius.spacing
                                }
                                text: "Use Patch Radius"
                                checked: true
                            }
                            DoubleSpinBox
                            {
                                id:patchRadius
                                label:"path radius"
                                min:0
                            }
                            DoubleSpinBox
                            {
                                id:surfaceRadius
                                label:"Surface Radius"
                                min:0
                            }

                            GroupBox {
                                title: "ODE Torsional Friction"
                                Layout.fillWidth: true

                                background: Rectangle {
                                    color: plt.background3
                                    radius: 7
                                }
                                label: Label {
                                    text: parent.title
                                    color: plt.textColor
                                    font.bold: true
                                }

                                ColumnLayout {
                                    spacing: 5
                                    DoubleSpinBox
                                    {
                                        id:torsionalSlip
                                        label:"Torsional Slip"
                                        min:0
                                    }


                                }
                            }
                        }
                    }
                    StackView
                    {
                        id: frictionSolverStack
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        initialItem: frictionOde
                    }

                    // ODE Friction


                    // Bullet Friction

                }
            }
            GroupBox {
                title: "Contact Properties"
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

                RowLayout{
                    spacing: 5

                    ColumnLayout{
                        spacing:5
                        CheckBox {
                            id: collideWithoutContact
                            text: "Collide Without Contact"
                            contentItem: Text
                            {
                                text:collideWithoutContact.text
                                color:plt.textColor
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: collideWithoutContact.indicator.width + collideWithoutContact.spacing
                            }
                        }
                        CustomSpinBox
                        {
                            id:collideWithoutContactBitmask
                            label:"Collide Without Contact Bitmask"

                        }

                        CustomSpinBox
                        {
                            id:collideBitmask
                            label:"Collide Bitmask"
                        }

                        CustomSpinBox
                        {
                            id:categoryBitmask
                            label:"Category Bitmask"
                        }
                        DoubleSpinBox{
                            id:poissonsRatio
                            label:"Poisons Ratio"
                        }
                        DoubleSpinBox
                        {
                            id:elasticModulus
                            label:"Young's Modulus"
                        }

                    }
                    StackView
                    {
                        id:contactSolverStack
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        initialItem: contactBullet
                    }

                    // ODE Contact

                    // Bullet Contact

                }
            }


            GroupBox {
                title: "Soft Contact Properties"
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
                clip: true

                    ColumnLayout {
                        spacing: 10

                        // DART Soft Contact Properties
                        GroupBox {
                            title: "DART"
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

                                // Bone Attachment
                                DoubleSpinBox {
                                    id: boneAttachment
                                    label: "Bone Attachment (k_v)"
                                    value: 100.0
                                    min: 0
                                    max: 1000

                                }

                                // Stiffness
                                DoubleSpinBox {
                                    id: stiffness
                                    label: "Stiffness (k_e)"
                                    value: 100.0
                                    min: 0
                                    max: 1000

                                }

                                // Damping
                                DoubleSpinBox {
                                    id: damping
                                    label: "Damping"
                                    value: 10.0
                                    min: 0
                                    max: 100

                                }

                                // Flesh Mass Fraction
                                DoubleSpinBox {
                                    id: fleshMassFraction
                                    label: "Flesh Mass Fraction"
                                    value: 0.05
                                    min: 0
                                    max: 1

                                }
                            }
                        }
                    }

            }
        }
    }


    Component{
        id:frictionOde
        GroupBox {
            title: "ODE Friction"
            Layout.fillWidth: true
            implicitWidth: frictionOdeLayout.width
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
                id:frictionOdeLayout
                spacing: 5
                DoubleSpinBox
                {
                    id:mu
                    label:"Mu"
                    min:0
                }
                DoubleSpinBox
                {
                    id:mu2
                    label:"Mu2"
                    min:0
                }
                Vect3
                {
                    id:fdir1
                    label:"fdir1"
                }

                DoubleSpinBox
                {
                    id:slip1
                    label:"slip1"
                    min:0
                }

                DoubleSpinBox
                {
                    id:slip2
                    label:"Slip2"
                    min:0
                }

            }
        }
    }
    Component{
        id:frictionBullet
        GroupBox {
            title: "Bullet Friction"
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
                DoubleSpinBox
                {
                    id:bulletFriction
                    label:"friction"
                    min:0
                }


                DoubleSpinBox
                {
                    id:bulletFriction2
                    label:"friction2"
                    min:0
                }

                Vect3
                {
                    id:bulletFdir1
                    label:"fdir1"

                }

                DoubleSpinBox
                {
                    id:rollingFriction
                    label:"Rolling friction"
                    min:0
                }

            }
        }
    }
    Component
    {
        id:contactOde
        GroupBox {
            title: "ODE Contact"
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
                DoubleSpinBox
                {
                    id:softCfm
                    label:"Soft cfm"
                    min:0
                }

                DoubleSpinBox
                {
                    id:softErp
                    label:"Soft Erp"
                    min:0
                }

                DoubleSpinBox
                {
                    id:kp
                    label:"Kp"
                    min:0
                }

                DoubleSpinBox
                {
                    id:kd
                    label:"Kd"
                    min:0
                }

                DoubleSpinBox
                {
                    id:maxVel
                    label:"Max vel"
                    min:0
                }

                DoubleSpinBox
                {
                    id:minDepth
                    label:"Min Depth"
                    min:0
                }
            }
        }

    }
    Component
    {
        id:contactBullet
        GroupBox {
            title: "Bullet Contact"
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
                DoubleSpinBox
                {
                    id:bulletSoftCfm
                    label:"Soft cfm"
                    min:0
                }

                DoubleSpinBox
                {
                    id:bulletSoftErp
                    label:"soft Erp"
                    min:0
                }

                DoubleSpinBox
                {
                    id:bulletKp
                    label:"Kp"
                    min:0
                }

                DoubleSpinBox
                {
                    id:bulletKd
                    label:"Kd"
                    min:0
                }



                CheckBox {
                    id: splitImpulse
                    text: "Split Impulse"
                    contentItem: Text
                    {
                        text:splitImpulse.text
                        color:plt.textColor
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: splitImpulse.indicator.width + splitImpulse.spacing
                    }
                    checked: true
                }
                DoubleSpinBox
                {
                    id:splitImpulseThreshold
                    label:"split Impulse Threshold"
                    min:0
                }


            }
        }

    }

}
