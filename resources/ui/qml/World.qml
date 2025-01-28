import QtQuick 2.9
import QtQuick.Window 2.0
import QtQuick.Controls 2.5
import QtQuick.Layouts 1.0
Rectangle{
    id:root
    property string textColor: plt.textColor
    property string textbackgroundColor: plt.textBackground
    property string backgroundColor:plt.background
    width: 700
    height:layout.height
    property int margin:16
    color:root.textbackgroundColor
    ColumnLayout
    {
        id:layout
        anchors.fill:parent
        spacing:24
        LineEdit
        {   textColor: root.textColor
            backgroundColor: root.textbackgroundColor
            id:worldname
            label:"WorldName"
            text:world.worldname
            Layout.margins:margin
            onTextvalChanged:
            {
                world.worldname=txt
            }
        }
        LineEdit
        {   textColor: root.textColor
            backgroundColor: root.textbackgroundColor
            id:robotname
            label:"RobotName"
            text:world.robotname
            Layout.margins:margin
            onTextvalChanged:
            {
                world.robotname=txt
            }

        }
        // Gravity groupbox

        // end of gravity groupbox
            Vect3
            {

                textcolor: root.textColor
                backgroundColor: root.backgroundColor
                textbackgroundColor:root.textbackgroundColor
                Layout.margins: margin
                label: "gravity"
                suffix: " m/s^2"
                min:-20
                max:20
                default_x:world.gravity[0]
                default_y:world.gravity[1]
                default_z:world.gravity[2]
                onXvalChanged:
                {
                    world.setGravity(xval,0)
                }
                onYvalChanged: {
                    world.setGravity(yval,1)
                }
                onZvalChanged:
                {
                    world.setGravity(zval,2)
                }
            }


        // magnetic Field
        Vect3
        {
            textcolor: root.textColor
            backgroundColor: root.backgroundColor
            textbackgroundColor:root.textbackgroundColor
            label:"MagneticField"
            Layout.margins: margin
            suffix: "T"
            dp:6
            min:-5
            max:4
            default_x:world.magneticField[0]
            default_y:world.magneticField[1]
            default_z:world.magneticField[2]
            onXvalChanged:
            {
                world.setMagneticField(xval,0)
            }
            onYvalChanged: {
                world.setMagneticField(yval,1)
            }
            onZvalChanged:
            {
                world.setMagneticField(zval,2)
            }

        }

        //wind
        Vect3
        {
            label:"Wind"
            textcolor: root.textColor
            backgroundColor: root.backgroundColor
            textbackgroundColor:root.textbackgroundColor
            Layout.margins: margin
            suffix: "m/s"
            default_x:world.wind_velocity[0]
            default_y:world.wind_velocity[1]
            default_z:world.wind_velocity[2]
            onXvalChanged:
            {
                world.setWind(xval,0)
            }
            onYvalChanged:
            {
                world.setWind(yval,1)
            }
            onZvalChanged:
            {
                world.setWind(zval,2)
            }
        }
        Rectangle
        {
            id:atm
            Layout.margins: margin
            radius:7
            color:root.backgroundColor
            width:glayout.implicitWidth+16
            height: glayout.implicitHeight+txt.implicitHeight+16
            Text{
                id:txt
                text: "Atmosphere"
                color: root.textColor
                font.pixelSize: 16
                anchors
                {
                    top:atm.top
                    topMargin:4
                    left:atm.left
                    leftMargin:8
                }
            }

            GridLayout
            {
                anchors
                {
                    top:txt.bottom
                    topMargin:4
                    left:txt.left
                }

                id:glayout
                columns:2
                rows: 2
                columnSpacing:56
                rowSpacing: 8
                RowLayout
                {
                    Text {
                        id: atmtype
                        text: qsTr("type")
                        color:root.textColor

                    }
                    ComboBox
                    {
                        model: ["adiabatic"]
                    }
                }
                DoubleSpinBox{
                    label:qsTr("Temperature")
                    suffix: "K"
                    min:-273
                    max:5000
                    decimalPlaces: 2
                    textcolor: root.textColor
                    backgroundColor: root.textbackgroundColor
                    default_value:world.temperature
                    onSpinBoxvalueChanged:
                    {
                        world.temperature=val
                    }
                }
                DoubleSpinBox
                {
                    label:qsTr("pressure")
                    suffix:"P"
                    min:0
                    max:1000000
                    textcolor: root.textColor
                     backgroundColor: root.textbackgroundColor
                    decimalPlaces: 2
                    default_value:world.pressure
                    onSpinBoxvalueChanged:
                    {
                        world.pressure=val
                    }
                }
                DoubleSpinBox
                {
                    textcolor: root.textColor
                     backgroundColor: root.textbackgroundColor
                    label:"Temperature\nGradient"
                    suffix: "K/m"
                    max:10
                    min:-10
                    decimalPlaces: 6
                    default_value:world.temperatureGrad
                    onSpinBoxvalueChanged:
                    {
                        world.temperatureGrad=val
                    }
                }
            }
        }
    }
}
