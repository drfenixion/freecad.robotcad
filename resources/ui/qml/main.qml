import QtQuick 2.9
import QtQuick.Window 2.0
import QtQuick.Controls 2.5
import QtQuick.Layouts 1.0
// use ApplicationWindow instead of QQwidget update will be done
Rectangle{
    visible: true
    width: 800
    height: 600


    // TabBar with two tabs: World and Physics
    TabBar {
        id: tabBar
        width: parent.width
        currentIndex: swipeView.currentIndex

        TabButton {
            text: "World"
        }
        TabButton {
            text: "Physics"
        }
    }

    // SwipeView to handle tab content
    SwipeView {
        id: swipeView
        anchors {
            top: tabBar.bottom
            bottom: parent.bottom
            left: parent.left
            right: parent.right
        }
        currentIndex: tabBar.currentIndex

        // World Tab (your previous layout)
        World
        {

        }

        // Physics Tab (empty for now)
        Physics
        {

        }
    }
}
