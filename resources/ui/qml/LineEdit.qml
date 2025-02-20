import QtQuick 2.9
import QtQuick.Layouts 1.0

FocusScope {
    property string textColor: "green"
    property string backgroundColor: "blue"
    property alias label:label.text
    property alias text:ti.text
    signal textvalChanged(string txt)
    id: root
    RowLayout
    {
        Text {
            id: label
            text: qsTr("text")
            height:ti.height
            font.pixelSize: ti.font.pixelSize
            color:root.textColor
        }
        spacing: 45
        Rectangle
        {

            height:30
            width: ti.implicitWidth < 70 ? 70 :ti.implicitWidth
            implicitWidth: ti.implicitWidth
            border.color: "#515dd0"
            color:root.backgroundColor
            radius: 4
            smooth:true
            TextInput{
                id:ti
                text:"none"
                color: root.textColor
                height: parent.height
                font.pixelSize: 24
                focus:true
                onTextChanged:
                {
                    root.textvalChanged(text)
                }
                
            }



        }
    }
}
