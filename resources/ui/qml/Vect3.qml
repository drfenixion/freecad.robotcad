import QtQuick 2.9
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.5

Item {
 // create 3 double spin boxes with lables x,y,z
    property alias label:template.text
    property int min: 0
    property int  max: 100

    width: gb.implicitWidth
    height:gb.implicitHeight
    GroupBox
    {
        id:gb
        Layout.margins: 10
        label:CheckBox{
            id:template
            checked:true
            text:qsTr("template")
            contentItem: Text
            {
                text:template.text
                color:"black"
                opacity: enabled ? 1:0.6
                verticalAlignment: Text.AlignVCenter
                leftPadding: template.indicator.width + template.spacing
            }

        }
        spacing:3
        RowLayout
        {
            id:layout
            enabled:template.checked
            anchors.fill: parent

            DoubleSpinBox
            {
                id:x
                label:qsTr("x")
                min:min
                max:max
            }
            DoubleSpinBox
            {
                id:y
                label: qsTr("y")
                min:min
                max:max
            }
            DoubleSpinBox
            {
                id:z
                label: qsTr("z")
                min:min
                max:max
            }
        }

    }
}
