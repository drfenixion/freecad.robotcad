import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GroupBox {
    title: "Inertial Properties"
    label: Label {
           text: parent.title
           color: plt.textColor // Change this to the desired color
           font.bold: true // Optional: Customize font properties
       }
    Layout.fillWidth: true
    background: Rectangle {
        color: plt.background1
        radius: 7
    }
    ColumnLayout {
        spacing: 12
        GridLayout{
            rows:2
            columns:2
            CheckBox {
                id: customMass
                text: "Use Custom Mass values"
                checked: prop.getter("CalculateInertiaBasedOnMass")
                onCheckStateChanged:
                {
                    if(checked)
                    {
                        prop.setter("CalculateInertiaBasedOnMass",1)
                        prop.setter("MaterialNotCalculate",1)
                    }
                    else
                    {
                       prop.setter("CalculateInertiaBasedOnMass",0)
                    }
                }

                contentItem: Text
                {
                    text:customMass.text
                    color:plt.textColor
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: customMass.indicator.width + customMass.spacing

                }
            }
            CheckBox {
                id: custominertia
                text: "Use Custom Inertia values"
                checked:prop.getter("MaterialNotCalculate")
                onCheckStateChanged:
                {
                    if(checked)
                    {
                        prop.setter("MaterialNotCalculate",1)
                    }
                    else
                    {
                       prop.setter("MaterialNotCalculate",0)
                    }
                }
                contentItem: Text
                {
                    text:custominertia.text
                    color:plt.textColor
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: custominertia.indicator.width + custominertia.spacing

                }
            }
            CheckBox {
                id: autoInertia
                text: "Automatic Inertia Calculation"
                contentItem: Text
                {
                    text:autoInertia.text
                    color:plt.textColor
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: autoInertia.indicator.width + autoInertia.spacing

                }
                onCheckStateChanged:{
                    if(checked)
                    {
                         prop.setter("MaterialNotCalculate",0)
                    }
                }
            }
            ButtonGroup {
                        id: buttonGroup
                        exclusive: true
                        buttons: [customMass, custominertia, autoInertia]
                    }
        }


        DoubleSpinBox{
            id: mass
            label: "Mass"
            min:0
            Layout.fillWidth: true
            // this will require its own property class
            // so as to allow conversion of all mass entered to the values accepted
            // by freecad e.g kg  to g
            enabled: customMass.checked


        }

        GroupBox {
            title: "Inertia"
            Layout.fillWidth: true
            enabled: custominertia.checked
            background: Rectangle {
                color: plt.background2
                radius: 7
            }
            label: Label {
                   text: parent.title
                   color: plt.textColor // Change this to the desired color
                   font.bold: true // Optional: Customize font properties
               }
            GridLayout {
                columns: 3
                rowSpacing: 12
                columnSpacing: 5

                DoubleSpinBox{
                    id:ixx
                    label:"Ixx"
                    min:0
                    default_value: prop.getter("Ixx")
                    onSpinBoxvalueChanged: {
                        prop.setter("Ixx",val)
                    }
                }

                DoubleSpinBox
                {
                    label:"Ixy"
                    id:ixy
                    min:0
                    default_value: prop.getter("Ixy")
                    onSpinBoxvalueChanged:
                    {
                        prop.setter("Ixy",val)
                    }
                }

               DoubleSpinBox
               {
                   id:ixz
                   label:"ixz"
                   min:0
                   default_value: prop.getter("Ixz")
                   onSpinBoxvalueChanged:
                   {
                       prop.getter("Ixz",val)
                   }
               }
                DoubleSpinBox{
                    id:iyy
                    label:"iyy"
                    min:0
                    default_value: prop.getter("Iyy")
                    onSpinBoxvalueChanged: {
                        prop.setter("Iyy",val)
                    }
                }

                DoubleSpinBox{
                    id:iyz
                    label:"iyz"
                    min:0
                    default_value: prop.getter("Iyz")
                    onSpinBoxvalueChanged: {
                        prop.setter("Iyz",val)
                    }
                }

                DoubleSpinBox
                {
                    id:izz
                    label:"izz"
                    min:0
                    default_value: prop.getter("Izz")
                    onSpinBoxvalueChanged: {
                        prop.setter("Izz",val)
                    }
                }
            }
        }

        // GroupBox {
        //     title: "Fluid Added Mass"
        //     Layout.fillWidth: true
        //     label: Label {
        //            text: parent.title
        //            color: plt.textColor // Change this to the desired color
        //            font.bold: true // Optional: Customize font properties
        //        }
        //     background: Rectangle {
        //         color: plt.background2
        //         radius: 7
        //     }
        //     GridLayout {
        //         columns: 3
        //         rowSpacing: 5
        //         columnSpacing: 5

        //         TextField { id: xx; placeholderText: "xx"; validator: DoubleValidator {} }
        //         TextField { id: xy; placeholderText: "xy"; validator: DoubleValidator {} }
        //         TextField { id: xz; placeholderText: "xz"; validator: DoubleValidator {} }
        //         TextField { id: xp; placeholderText: "xp"; validator: DoubleValidator {} }
        //         TextField { id: xq; placeholderText: "xq"; validator: DoubleValidator {} }
        //         TextField { id: xr; placeholderText: "xr"; validator: DoubleValidator {} }
        //         TextField { id: yy; placeholderText: "yy"; validator: DoubleValidator {} }
        //         TextField { id: yz; placeholderText: "yz"; validator: DoubleValidator {} }
        //         TextField { id: yp; placeholderText: "yp"; validator: DoubleValidator {} }
        //         TextField { id: yq; placeholderText: "yq"; validator: DoubleValidator {} }
        //         TextField { id: yr; placeholderText: "yr"; validator: DoubleValidator {} }
        //         TextField { id: zz; placeholderText: "zz"; validator: DoubleValidator {} }
        //         TextField { id: zp; placeholderText: "zp"; validator: DoubleValidator {} }
        //         TextField { id: zq; placeholderText: "zq"; validator: DoubleValidator {} }
        //         TextField { id: zr; placeholderText: "zr"; validator: DoubleValidator {} }
        //         TextField { id: pp; placeholderText: "pp"; validator: DoubleValidator {} }
        //         TextField { id: pq; placeholderText: "pq"; validator: DoubleValidator {} }
        //         TextField { id: pr; placeholderText: "pr"; validator: DoubleValidator {} }
        //         TextField { id: qq; placeholderText: "qq"; validator: DoubleValidator {} }
        //         TextField { id: qr; placeholderText: "qr"; validator: DoubleValidator {} }
        //         TextField { id: rr; placeholderText: "rr"; validator: DoubleValidator {} }
        //     }
        // }
    }
}
