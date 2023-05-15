import QtQuick 2.0
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 600
    height: 400
    title: "Multi-agent system visualization"

    GridLayout {
        columns: 2
        width: parent.width
        height: parent.height

        GLMakiePlot {
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.columnSpan: 2
        }

        Text {
            text: "Robot information"
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

        GLMakiePlot {
            Layout.fillHeight: true
            Layout.fillWidth: true
        }
    }
}
