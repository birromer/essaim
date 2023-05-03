using QML

# Define the QML code for the interface
qml_code = """
import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtCharts 2.3

Window {
    visible: true
    width: 800
    height: 600

    // Define the robots view
    Rectangle {
        anchors.fill: parent
        anchors.margins: 10

        // Define the background
        Rectangle {
            anchors.fill: parent
            color: "#F7F7F7"
        }

        // Define the robot items
        Repeater {
            model: robot_positions

            Rectangle {
                id: robot
                width: 30
                height: 30
                color: "blue"
                x: modelData[1]
                y: modelData[2]

                // Rotate the robot based on its heading
                rotation: -modelData[3] * 180 / Math.PI
            }
        }
    }

    // Define the robot status view
    Rectangle {
        anchors.top: robots_view.bottom
        anchors.left: parent.left
        width: parent.width / 3
        height: parent.height / 3

        // Define the background
        Rectangle {
            anchors.fill: parent
            color: "#F7F7F7"
        }

        // Define the status items
        Repeater {
            model: robot_statuses

            Text {
                id: status
                text: modelData
                y: index * 30
            }
        }
    }

    // Define the adjacency matrix view
    Rectangle {
        anchors.top: robot_status_view.bottom
        anchors.left: parent.left
        width: parent.width / 3
        height: parent.height / 3

        // Define the background
        Rectangle {
            anchors.fill: parent
            color: "#F7F7F7"
        }

        // Define the adjacency matrix
        ChartView {
            anchors.fill: parent
            theme: ChartView.ChartThemeDark

            ValueAxis {
                id: xAxis
                min: 1
                max: num_robots
                tickCount: num_robots
                labelFormat: "%d"
            }

            ValueAxis {
                id: yAxis
                min: 1
                max: num_robots
                tickCount: num_robots
                labelFormat: "%d"
                reverse: true
            }

            ScatterSeries {
                id: adjacency_matrix
                name: "Adjacency Matrix"
                markerSize: 15

                function getData() {
                    var data = []
                    for (var i = 0; i < num_robots; i++) {
                        for (var j = 0; j < num_robots; j++) {
                            if (adj_matrix[i][j] == 1) {
                                data.push({"x": i+1, "y": j+1})
                            }
                        }
                    }
                    return data
                }

                property int num_robots: adjacency_matrix_data.length
                property var adjacency_matrix_data: getData()
                XYPoint { x: modelData.x; y: modelData.y }
            }
        }
    }
}
"""

# Define the robot positions, statuses, and adjacency matrix
robot_positions = [
  (100, 100, 0.0),
  (200, 200, 1.2),
  (300, 300, 1.2),
  (400, 400, 2.4),
  (500, 500, 3.6)
]

robot_statuses = ["Robot 1: Moving", "Robot 2: Idle", "Robot 3: Stopped", "Robot 4: Moving", "Robot 5: Moving"]

adj_matrix = [ 0 1 0 0 0;
               1 0 1 0 0;
               0 1 0 1 0; 
               0 0 1 0 1;
               0 0 0 1 0 ] 

# Create the QML engine and set the context properties
engine = QML.QQmlApplicationEngine()
engine[:rootContext]()[:robot_positions] = robot_positions
engine[:rootContext]()[:robot_statuses] = robot_statuses
engine[:rootContext]()[:adj_matrix] = adj_matrix

# Load the QML code and start the application
component = QQmlComponent(engine)
component[:setData](qml_code, "")
window = component[:create]()
exec(Events())
