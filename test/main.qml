import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3

ApplicationWindow {
    visible: true
    width: 600
    height: 400
    title: "Robot Simulation"

    SplitView {
        id: splitView
        anchors.fill: parent

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 2 / 3 * parent.width

            /* Here, you would use the QML Makie plot. Unfortunately, as of my knowledge cutoff in September 2021,
            there isn't direct support for embedding Makie plots in a QML application. You might need to
            save your plot as an image file and display that, or find another solution. */

            /* Placeholder: */
            Rectangle {
                anchors.fill: parent
                color: "lightblue"
            }
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 1 / 3 * parent.width

            Rectangle {
                id: upperRectangle
                color: "lightgreen"
                width: parent.width
                height: parent.height / 2

                /* Here, you would display the robot positions and orientations. */
            }

            Rectangle {
                id: lowerRectangle
                color: "lightyellow"
                width: parent.width
                height: parent.height / 2

                /* Here, you would display the visibility graph. */
            }
        }
    }
}
