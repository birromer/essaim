import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    width: 800
    height: 600

    Column {
        anchors.fill: parent

        Row {
            anchors.fill: parent
            Rectangle {
                id: robotDisplay
                width: parent.width * 2 / 3
                height: parent.height
                color: "lightgray"

                // Add code to display robots moving
            }

            Column {
                width: parent.width / 3
                height: parent.height

                Rectangle {
                    id: stateDisplay
                    width: parent.width
                    height: parent.height / 2
                    color: "lightblue"

                    // Add code to display current state of each robot
                }

                Rectangle {
                    id: graphDisplay
                    width: parent.width
                    height: parent.height / 2
                    color: "lightgreen"

                    // Add code to display adjacency matrix
                }
            }
        }
    }
}
