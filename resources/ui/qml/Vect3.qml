import QtQuick 2.9
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.5

Item {
 // create 3 double spin boxes with lables x,y,z
    id:root
    property string textcolor:"white"
    property string textbackgroundColor: "green"
    property string backgroundColor: "black"
    property alias label:gb.title
    property real default_x:0.0
    property real default_y:0.0
    property real default_z:0.0
    property int dp:3
    property int min: 0
    property int  max: 100
    signal xvalChanged(real xval)
    signal yvalChanged(real yval)
    signal zvalChanged(real zval)
    property string suffix:""
    width: gb.implicitWidth+16
    height:gb.implicitHeight+16
    GroupBox
    {   id:gb
        title: "Default"
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


        RowLayout
        {
            id:layout
            spacing: 10
            anchors.fill: parent
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
                default_value:root.default_x
                onSpinBoxvalueChanged:
                {
                    root.xvalChanged(val)
                }
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
                 default_value:root.default_y
                 onSpinBoxvalueChanged:
                 {
                     root.yvalChanged(val)
                 }
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
                 default_value:root.default_z
                 onSpinBoxvalueChanged:
                 {
                     root.zvalChanged(val)
                 }
            }
        }

    }
}
