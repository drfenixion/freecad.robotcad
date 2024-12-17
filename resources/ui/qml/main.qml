import QtQuick 2.9
import QtQuick.Window 2.0
import QtQuick.Controls 2.5 
import QtQuick.Layouts 1.0
Item {
    width: 700
    height:layout.height
    visible: true


    property int margin:10
    ColumnLayout
    {
        id:layout
        anchors.fill:parent
        spacing:3
        LineEdit
        {
            id:worldname
            label:"Name"
            Layout.margins:margin
        }
        // Gravity groupbox

        // end of gravity groupbox
            Vect3
            {
                Layout.margins: margin
                label: "gravity"
                min:-20
                max:20
            }

        // magnetic Field
        Vect3
        {
            label:"MagneticField"
            Layout.margins: margin

        }

        //wind
        Vect3
        {
            label:"Wind"
            Layout.margins: margin
        }
        GroupBox
        {
            id:atm
            Layout.margins: margin
            label:CheckBox
            {
            id:template
            checked:true
            text:qsTr("Atmosphere")
            contentItem: Text
            {
                text:template.text
                color:"black"
                opacity: enabled ? 1:0.6
                verticalAlignment: Text.AlignVCenter
                leftPadding: template.indicator.width + template.spacing
            }
            }
            GridLayout
            {
                anchors.fill:parent
                columns:2
                rows: 2
                RowLayout
                {
                    Text {
                        id: atmtype
                        text: qsTr("type")

                    }
                    ComboBox
                    {
                        model: ["adiabatic"]
                    }
                }
                DoubleSpinBox{
                    label:qsTr("Temperature")
                }
                DoubleSpinBox
                {
                    label:qsTr("pressure")
                }
                DoubleSpinBox
                {
                    label:"Temperature\nGradient"
                }
            }
        }
    }
}
