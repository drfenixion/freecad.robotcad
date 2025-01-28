import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    visible: true
    width: 800
    height: 600
    color:plt.textBackground
    ColumnLayout {
        anchors.fill: parent
        spacing: 10
         Layout.leftMargin: 30
         Layout.rightMargin: 12
         Layout.topMargin: 10
         Layout.bottomMargin: 12
        // General Physics Settings (Outside StackView)
        Rectangle{
            color:plt.background
            radius: 7
            width:general_grid.implicitWidth+70
            height:250

            ColumnLayout {
                id:general_grid
                anchors.fill: parent
                spacing: 20
                Layout.leftMargin: 30
                Layout.rightMargin: 12
                Layout.topMargin: 10
                Layout.bottomMargin: 12
                RowLayout{
                    Label { text: "Type:" }
                    ComboBox {
                        id: physicsType
                        model: ["ode", "bullet", "simbody", "dart"]
                        currentIndex: 0
                        width: 200
                        onCurrentIndexChanged: updateSolverView()
                    }
                }
                DoubleSpinBox{
                    label:"max step size"
                    decimalPlaces:5

                }
                DoubleSpinBox
                {
                    label:"Real Time Factor"
                    decimalPlaces: 2

                }

                DoubleSpinBox{
                    label:"max Contacts"
                    decimalPlaces: 2
                }

            }
        }

        // StackView to Hold Solver-Specific Settings


        StackView {
            id: stackView
            //Layout.fillWidth: true
           Layout.fillHeight: true
            width:600


            initialItem: odeSettings // Set ODE as the default solver
        }

    }



    // Function to Update Solver View Based on ComboBox Selection
    function updateSolverView() {
        switch (physicsType.currentIndex) {
        case 0: // ODE
            stackView.replace(odeSettings);
            break;
        case 1: // Bullet
            stackView.replace(bulletSettings);
            break;
        case 2: // Simbody
            stackView.replace(simbodySettings);
            break;
        case 3: // DART
            stackView.replace(dartSettings);
            break;
        default:
            stackView.replace(odeSettings); // Fallback to ODE
        }
    }

    // ODE Settings Component
    Component {
        id: odeSettings
        Rectangle{
            radius: 7
            width: stackView.width
            color: plt.background
            ColumnLayout {
                width: parent.width
                spacing: 12
                RowLayout
                {

                    Label { text: "Solver Type:" }
                    ComboBox {
                        id:stcb
                        model: ["world", "quick"]
                        currentIndex: 0
                        width: 200
                    }
                }

                DoubleSpinBox{
                    label:"min Step Size"
                    decimalPlaces: 5
                }
                //int
                RowLayout{
                    Label { text: "Island Threads:" }
                    SpinBox{
                        value: 0
                    }
                }

                //int
                RowLayout{
                    Label { text: "Iterations:" }
                    SpinBox{
                        value: 50
                    }
                }

                DoubleSpinBox{
                    label:"Sor"
                    value:1.3
                }

                // Constraints Settings
                Rectangle {
                    //title: "Constraints"
                    Layout.columnSpan: 2
                    Layout.fillWidth: true

                    ColumnLayout {
                        spacing:12
                        width: parent.width
                        //int
                        DoubleSpinBox{
                            label:"CFM"
                            value:0.0
                            decimalPlaces: 3
                        }

                        DoubleSpinBox{
                            label:"ERP"
                            value: 0.2
                            decimalPlaces: 3
                        }

                        //
                        DoubleSpinBox{
                            label:"Contact Max Correcting Vel"
                            decimalPlaces: 2

                        }

                       DoubleSpinBox{
                           label:"Contact Surface Layer"
                           decimalPlaces:4
                       }


                    }
                }
            }
        }
    }

    // Bullet Settings Component
    Component {
        id: bulletSettings
        Rectangle {
                 color: plt.background
                 radius:7
            width: stackView.width

            ColumnLayout {

                width: parent.width

                RowLayout
                {
                    Label { text: "Solver Type:" }
                    ComboBox {
                        model: ["sequential_impulse"]
                        currentIndex: 0
                        width:400
                    }
                }
                DoubleSpinBox
                {
                    label:"Min Step Size"
                    decimalPlaces: 5
                }



               RowLayout{
                   Label
                   {
                   text:"Iterations"
                   }
                   SpinBox
                   {
                       value:50
                   }
               }

                DoubleSpinBox
                {
                    label:"Sor"
                    decimalPlaces: 4
                }

                // Constraints Settings
                Label{
                    text:"Constraints"
                }

                Rectangle {

                    color:plt.background
                    Layout.fillWidth: true


                    ColumnLayout {

                        width: parent.width

                        RowLayout
                        {
                            Label{
                                text:"CFM"
                            }
                            SpinBox{
                                value:0
                            }

                        }

                       DoubleSpinBox{
                           label: "ERP"
                           decimalPlaces: 3
                       }
                    DoubleSpinBox{
                        label:"Contact Surface Layer"
                        decimalPlaces: 4
                        value:0.001
                    }


                       RowLayout
                       {

                           Label { text: "Split Impulse:" }
                           CheckBox { checked: true }

                       }
                       DoubleSpinBox
                       {
                           label: "Split impulse Penetration Threshold"
                           value:-0.1
                           min: -1
                           decimalPlaces: 3
                       }
                    }
                }
            }
        }
    }

    // DART Settings Component
    Component {
        id: dartSettings
        Rectangle {
            //title: "DART Physics Settings"
            width: stackView.width
            radius: 7
            color:plt.background
            ColumnLayout {

                width: parent.width

                RowLayout{
                    Label { text: "Solver Type:" }
                    ComboBox {
                        model: ["dantzig", "pgs"]
                        currentIndex: 0
                        width:300
                    }
                }

                RowLayout
                {
                    Label { text: "Collision Detector:" }
                    ComboBox {
                        model: ["fcl", "dart", "bullet", "ode"]
                        currentIndex: 0
                        width:300
                    }
                }
            }
        }
    }

    // Simbody Settings Component
    Component {
        id: simbodySettings
        Rectangle {
            //title: "Simbody Physics Settings"
            width: stackView.width
            color:plt.background
            radius:7
            ColumnLayout {

                width: parent.width
                spacing:10
                DoubleSpinBox{
                    label:"Min Step Size"
                    value:0.0001
                    decimalPlaces: 4
                }
                DoubleSpinBox
                {
                    label:"Accuracy"
                    decimalPlaces: 4
                    value:1e-3
                }

               DoubleSpinBox{
                   label:"Max Transient Velocity"
                   decimalPlaces: 3
                   value:0.01
               }
                // Contact Settings
                Rectangle {
                    //title: "Contact Settings"
                    color: plt.background
                    Layout.fillWidth: true

                    ColumnLayout {
                        spacing: 10
                        width: parent.width
                        DoubleSpinBox{
                            label:"Stiffnesss"
                            value:1e8
                            decimalPlaces: 7
                        }



                        RowLayout{
                            Label { text: "Dissipation:" }
                            SpinBox
                            {
                                value:100
                            }
                        }
                        DoubleSpinBox{
                            label:"Plastic Coef Restitution"
                            decimalPlaces: 3
                            value:0.5
                        }

                        DoubleSpinBox{
                            label:"Plastic Impact Velocity"
                            decimalPlaces: 2
                            value: 0.5
                        }
                        DoubleSpinBox{
                            label:"Static Friction"
                            decimalPlaces: 2
                            value:0.9
                        }

                        DoubleSpinBox{
                            label:"Dynamic Friction"
                            decimalPlaces: 2
                            value: 0.9
                        }

                        DoubleSpinBox{
                            label:"Viscous Friction"
                            decimalPlaces: 2
                            value:0.0
                        }
                    }
                }
            }
        }
    }
}
