import QtQuick 2.9
import QtQuick.Layouts 1.0
FocusScope {
    property alias label:label.text
    property string text:ti.text
    id: name
    RowLayout
    {
        Text {
            id: label
            text: qsTr("text")
            height:ti.height
            font.pixelSize: ti.font.pixelSize
        }
        spacing: 45
        Rectangle
        {

            height:30
            width: ti.implicitWidth < 70 ? 70 :ti.implicitWidth
            implicitWidth: ti.implicitWidth
            border.color: "#404ec5"
            color:"white"
            radius: 4
            smooth:true
            TextInput{
                id:ti
                text:"enter Text"
                color: focus ? "black":"gray"
                height: parent.height
                font.pixelSize: 24
                focus:true
            }



        }
    }
}
