import QtQuick 2.9
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.5

Item {
 // create 3 double spin boxes with lables x,y,z
    id:root
    property string textcolor:"white"
    property string textbackgroundColor: "green"
    property string backgroundColor: "black"
    property alias label:lbl.text
    property int dp:3
    property int min: 0
    property int  max: 100
    property string suffix:""
    width: gb.implicitWidth+16
    height:gb.implicitHeight+16
    Rectangle
    {
        radius: 7
        id:gb
        color:root.backgroundColor
        width: layout.implicitWidth+16
        height:layout.implicitHeight+lbl.implicitHeight+30
        Layout.margins: 12
        Text {
            id: lbl
            text: qsTr("text")
            color:root.textcolor
            font.pixelSize: 16
            anchors{
                margins: 12
                leftMargin: 12
                top:gb.top
                topMargin: 2
                left:gb.left

            }
        }


        RowLayout
        {
            id:layout
            spacing: 10

            anchors
            {
                left:lbl.left
                top:lbl.bottom
                topMargin:7
            }

            DoubleSpinBox
            {
                id:x
                label:qsTr("x")
                suffix:root.suffix
                textcolor: root.textcolor
                backgroundColor: root.textbackgroundColor
                min:root.min
                max:root.max
                decimalPlaces: root.dp
            }
            DoubleSpinBox
            {
                id:y
                label: qsTr("y")
                textcolor: root.textcolor
                backgroundColor: root.textbackgroundColor
                min:root.min
                max:root.max
                 suffix:root.suffix
                 decimalPlaces: root.dp
            }
            DoubleSpinBox
            {
                id:z
                textcolor: root.textcolor
                backgroundColor: root.textbackgroundColor
                label: qsTr("z")
                min:root.min
                max:root.max
                 suffix:root.suffix
                 decimalPlaces: root.dp
            }
        }

    }
}
