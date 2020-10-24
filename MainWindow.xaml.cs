using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace GraphShortestPath
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //Global variables 
        byte[,] graph_unweighted = new byte[9, 9];
        byte[,] graph_weighted = new byte[9, 9];
        List<byte> path = new List<byte>();
        byte shortestPathLength = INFINITY;

        const byte INFINITY = byte.MaxValue;

        public MainWindow()
        {
            InitializeComponent();

            //set width and height of the canvas according to screen resolution
            canvasDisplay.Height = System.Windows.SystemParameters.PrimaryScreenHeight - 200;
            canvasDisplay.Width = System.Windows.SystemParameters.PrimaryScreenWidth - 50;

            //populate graph matrix
            graph_unweighted = Generate9x9_Matrix();
            graph_weighted = Generate9x9_WeightedMatrix();
        }

        /// <summary>
        /// Amar Shrestha - 10/3/2020
        /// Generates 9x9 array with 30% filled with 1 and rest with 0
        /// Row represents "from" and column represents "to" in graph
        /// 1 reprents presence of path and 0 represents absence of path
        /// </summary>
        /// <returns></returns>
        private byte[,] Generate9x9_Matrix()
        {
            byte[,] array = new byte[9, 9];
            float percent = 0;
            int seeder = 1;

            //loop until 30% of the matrix is populated
            while (percent < 29.5)
            {
                Random random1 = new Random((int)(DateTime.Now.Ticks) + seeder++);
                Random random2 = new Random((int)(DateTime.Now.Ticks) + 100 * seeder++);

                //randomly selects row and column index
                int row = random1.Next(9);
                int col = random2.Next(9);

                //populates the cell with 1 if the cell is previously empty  and is not a diagonal element
                if (array[row, col] == 0 && row != col)
                {
                    array[row, col] = 1;
                    //update the percentage filled
                    percent += (1 / (float)array.Length) * 100;
                }
            }

            return array;
        }

        /// <summary>
        /// Amar Shrestha - 10/3/2020
        /// Returns string format of the matrix
        /// </summary>
        /// <param name="array"></param>
        private string StringFormatted9x9_Array(byte[,] array)
        {
            string matrix = "       1     2     3     4     5     6     7     8     9\n";
            matrix += "---------------------------------------------\n";
            for (int i = 0; i <= array.GetUpperBound(0); i++)
            {
                matrix += i + 1 + "  |  ";
                for (int j = 0; j <= array.GetUpperBound(1); j++)
                {
                    matrix += array[i, j] + (array[i, j].ToString().Length > 1 ? "   " : "     ");
                }
                matrix += "\n";
            }
            matrix += "---------------------------------------------\n";

            return matrix;
        }

        /// <summary>
        /// Amar Shrestha - 10/3/2020
        /// Draws a circle in canvas
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="radius"></param>
        private void DrawCircle(float x, float y, int radius)
        {

            Ellipse circle = new Ellipse()
            {
                Width = radius * 2,
                Height = radius * 2,
                Stroke = Brushes.DarkGreen,
                Fill = Brushes.Green,
                StrokeThickness = 3
            };

            canvasDisplay.Children.Add(circle);

            //set coordinates in canvas
            circle.SetValue(Canvas.LeftProperty, (double)x);
            circle.SetValue(Canvas.TopProperty, (double)y);
        }

        /// <summary>
        /// Amar Shrestha - 10/3/2020
        /// Draws a node with circle and node number on center
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="radius"></param>
        /// <param name="text"></param> 
        private void DrawNode(float x, float y, string text, int radius = 25)
        {
            Label label = new Label();
            label.VerticalAlignment = VerticalAlignment.Center;
            label.HorizontalAlignment = HorizontalAlignment.Center;
            label.VerticalContentAlignment = VerticalAlignment.Center;
            label.HorizontalContentAlignment = HorizontalAlignment.Center;
            label.FontFamily = new FontFamily("Verdana");
            label.FontSize = 25;
            label.Foreground = Brushes.White;

            DrawCircle(x, y, radius);

            label.Content = text;
            canvasDisplay.Children.Add(label);

            //set coordinates in canvas so that the lable is in the center of the circle
            label.SetValue(Canvas.LeftProperty, (double)(x + radius / 2));
            label.SetValue(Canvas.TopProperty, (double)(y + radius / 5));
        }

        /// <summary>
        /// Amar Shrestha - 10/3/2020
        /// Draws a line in canvas
        /// </summary>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x2"></param>
        /// <param name="y2"></param>
        private void DrawLine(float x1, float y1, float x2, float y2)
        {
            Line line = new Line();
            Thickness thickness = new Thickness(0, 0, 0, 0);
            line.Margin = thickness;
            line.Visibility = Visibility.Visible;
            line.StrokeThickness = 4;

            //selects line color based on the coordinates
            SolidColorBrush brush = new SolidColorBrush(Color.FromArgb(255, (byte)((x1 + y1 * x1) % 256), (byte)((y1 * x1 + y2 * x2) % 256), (byte)((y2 + x2) % 256)));
            line.Stroke = brush;

            //cooridinates for the line
            line.X1 = x1;
            line.X2 = x2;
            line.Y1 = y1;
            line.Y2 = y2;

            //Draw the pointing arrow 
            DrawPointingArrow(x1, y1, x2, y2, 25, brush);

            canvasDisplay.Children.Add(line);
        }

        /// <summary>
        /// /// Amar Shrestha - 10/3/2020
        /// Draws the nodes and paths to each other based on the matrix.
        /// Also displays path weights if the matrix is weighted
        /// Displays either full graph or just the path from start node to end node
        /// </summary>
        /// <param name="array"></param>
        /// <param name="weighted"></param>
        /// <param name="path"></param>
        private void DrawGraph(byte[,] array, bool weighted = false, List<byte> path = null)
        {
            int radius = 25;
            float[] xCoord = new float[9];
            float[] yCoord = new float[9];

            //node 1
            xCoord[0] = (float)(canvasDisplay.Width / 2 - 30);
            yCoord[0] = 0;

            //node 2
            xCoord[1] = (float)(canvasDisplay.Width / 4);
            yCoord[1] = (float)(canvasDisplay.Height / 4 - 30);

            //node 3
            xCoord[2] = (float)(canvasDisplay.Width - (canvasDisplay.Width / 4));
            yCoord[2] = (float)(canvasDisplay.Height / 4 - 30);

            //node 4
            xCoord[3] = (float)(canvasDisplay.Width / 2 + 30);
            yCoord[3] = (float)(canvasDisplay.Height / 2 - 40);

            //node 5
            xCoord[4] = 30;
            yCoord[4] = (float)(canvasDisplay.Height / 2 + 30);

            //node 6
            xCoord[5] = (float)(canvasDisplay.Width - 30);
            yCoord[5] = (float)(canvasDisplay.Height / 2 + 31);

            //node 7
            xCoord[6] = (float)(canvasDisplay.Width / 4);
            yCoord[6] = (float)(canvasDisplay.Height - canvasDisplay.Height / 4 + 50);

            //node 8
            xCoord[7] = (float)(canvasDisplay.Width - canvasDisplay.Width / 4);
            yCoord[7] = (float)(canvasDisplay.Height - canvasDisplay.Height / 4 + 51);

            //node 9
            xCoord[8] = (float)(canvasDisplay.Width / 2 - 30);
            yCoord[8] = (float)(canvasDisplay.Height);


            //for (int i = 0; i < xCoord.Length; i++)
            //{
            //    Random rand = new Random((int)DateTime.Now.Ticks + i * 1000);
            //    xCoord[i] = rand.Next((int)canvasDisplay.Width);
            //}

            //for (int i = 0; i < xCoord.Length; i++)
            //{
            //    Random rand = new Random((int)DateTime.Now.Ticks + i * 100);
            //    yCoord[i] = rand.Next((int)canvasDisplay.Height);
            //}

            //path is null means show all the connections
            if (path == null)
            {
                for (int i = 0; i < xCoord.Length; i++)
                {
                    for (int j = 0; j < yCoord.Length; j++)
                    {
                        //only draw lines for paths with weight greater than zero
                        if (array[i, j] > 0)
                        {
                            DrawLine(xCoord[i], yCoord[i], xCoord[j], yCoord[j]);
                        }
                    }
                }
            }
            else
            {   //show only path from start node to end node
                for (int i = 0; i < path.Count - 1; i++)
                {
                    //'path' is sequential so the next vertex on path is simply the next element on 'path' hence path[i]->path[i+1]->path[i+2]...
                    DrawLine(xCoord[path[i]], yCoord[path[i]], xCoord[path[i + 1]], yCoord[path[i + 1]]);
                }
            }

            //show weights of the path
            if (weighted)
            {
                //path is null means show all the connections
                if (path == null)
                {
                    for (int i = 0; i < xCoord.Length; i++)
                    {
                        for (int j = 0; j < yCoord.Length; j++)
                        {
                            if (array[i, j] > 0)
                            {
                                DisplayPathWeights(xCoord[i], yCoord[i], xCoord[j], yCoord[j], array[i, j].ToString());
                            }
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < path.Count - 1; i++)
                    {
                        DisplayPathWeights(xCoord[path[i]], yCoord[path[i]], xCoord[path[i + 1]], yCoord[path[i + 1]], array[path[i], path[i + 1]].ToString());
                    }
                }
            }

            //Draws nodes in canvas
            for (int j = 0; j < xCoord.Length; j++)
            {
                DrawNode(xCoord[j] - radius, yCoord[j] - radius, (j + 1).ToString());
            }

        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Depth First Search algorithm
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="graph"></param>
        /// <returns></returns>
        public List<byte> DepthFirstSearch(byte start, byte end, byte[,] graph)
        {
            bool[] visited = new bool[9];
            List<byte> path = new List<byte>();

            //DFS solving algorithm
            DFS(start, end, graph, ref visited, ref path);

            //Trace back the path to contruct connected path
            List<byte> connectedPath = new List<byte>();
            for (int i = path.Count - 1; i >= 0; i--)
            {
                //finish tracing path when end is found
                if (path[i] == end)
                {
                    connectedPath.Add(end);
                    break;
                }
                connectedPath.Add(path[i]);
            }

            return connectedPath;
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Solving algorithm for depth first search
        /// Visits the first neighbor of each node and so on until it reaches the end of the graph and
        /// repeats the same process with every other node
        /// </summary>
        /// <param name="current"></param>
        /// <param name="end"></param>
        /// <param name="graph"></param>
        /// <param name="visited"></param>
        /// <param name="path"></param>
        private void DFS(byte current, byte end, byte[,] graph, ref bool[] visited, ref List<byte> path)
        {
            //return if the end is already visited
            if (visited[end]) return;

            //add to path and return if end is reached
            if (current == end)
            {
                visited[current] = true;
                path.Add(end);
                return;
            }

            //return if the current node is already visited
            if (visited[current]) return;

            //mark current note as visited
            visited[current] = true;

            //get the neighboring nodes of the current node
            List<byte> neighbors = GetNeighbors(graph, current);

            //iterate through all neighbors of current node and perform DFS on them as well
            foreach (byte index in neighbors)
            {
                DFS(index, end, graph, ref visited, ref path);
            }

            //add the current node in the path
            path.Add(current);
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Breadth First Search wrapper function
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="graph"></param>
        /// <returns></returns>
        private List<byte> BreadthFirstSearch(byte start, byte end, byte[,] graph)
        {
            //find the array of previous node using BFS
            short[] previousNodes = SolveBFS(start, graph);

            //string text = "";
            //for (int i = 0; i < previousNodes.Length; i++)
            //{
            //    text += previousNodes[i] + " ";
            //}
            //MessageBox.Show(text);

            //Trace the path from start to end node
            return TracePath_BFS(start, end, previousNodes);
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Breadth First Search Algorithm
        /// Visits every neighboring nodes of the current node before moving deeper and repeats this
        /// process until the end of graph is reached
        /// </summary>
        /// <param name="start"></param>
        /// <param name="graph"></param>
        /// <returns></returns>
        private short[] SolveBFS(byte start, byte[,] graph)
        {
            //adds the starting node in the queue
            Queue<byte> queue = new Queue<byte>();
            queue.Enqueue(start);

            bool[] visited = new bool[9];
            visited[start] = true;

            //array to store the link parent nodes
            short[] previousNodes = new short[9];

            //initiate all parent nodes to be -1
            for (int i = 0; i < previousNodes.Length; i++)
            {
                previousNodes[i] = -1;
            }

            //iterate the process until the queue is empty
            while (!queue.isEmpty())
            {
                //pop the last element on the queue
                byte node = queue.Dequeue();

                //get neighboring nodes of the given node
                List<byte> neighbors = GetNeighbors(graph, node);

                //iterate through all neighbors of the given nodes
                foreach (byte neighbor in neighbors)
                {
                    //if the given neighbor is not visited then add it to queue
                    if (!visited[neighbor])
                    {
                        queue.Enqueue(neighbor);
                        visited[neighbor] = true;
                        previousNodes[neighbor] = node;
                    }
                }
            }

            return previousNodes;
        }

        /// <summary>
        /// Amar Shrestha - 10/12/2020
        /// Traces the path from start to end for BFS
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="previousNodes"></param>
        /// <returns></returns>
        private List<byte> TracePath_BFS(byte start, byte end, short[] previousNodes)
        {
            List<byte> path = new List<byte>();

            //start from end and trace back and iterate using previousNodes
            //until -1 is reached because the parent of start node is -1
            for (short i = end; i != -1; i = previousNodes[i])
            {
                path.Add((byte)i);
            }

            //reverse the traced path so that start is at the beginning
            path.Reverse();

            //return the traced path only if the first element in path is start node
            if (path[0] == start)
            {
                return path;
            }

            //Otherwise return empty list
            path.Clear();
            return path;
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Gets the list of all the neighboring nodes for the given nodes with path weights greater than 0
        /// </summary>
        /// <param name="graph"></param>
        /// <param name="node"></param>
        /// <returns></returns>
        private List<byte> GetNeighbors(byte[,] graph, byte node)
        {
            List<byte> neighbors = new List<byte>();
            for (byte i = 0; i < graph.GetLength(0); i++)
            {
                //only add neighbors that has path weights greater than zero
                //do not add the the node itself as its neighbor
                if (graph[node, i] > 0 && node != i)
                {
                    neighbors.Add(i);
                }
            }
            return neighbors;
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Generates 9x9 matrix with weighted path 
        /// </summary>
        /// <returns></returns>
        private byte[,] Generate9x9_WeightedMatrix()
        {
            byte[,] array = new byte[9, 9];
            float percent = 0;
            int seeder = 1;

            //iterate until percentage is 30%
            while (percent < 29.5)
            {
                Random random1 = new Random((int)(DateTime.Now.Ticks) + seeder++);
                Random random2 = new Random((int)(DateTime.Now.Ticks) + 100 * seeder++);
                Random random = new Random((int)(DateTime.Now.Ticks) + 1000 * seeder++);

                //randomly selects rows and columns in matrix
                int row = random1.Next(9);
                int col = random2.Next(9);

                //add weights only if the node is empty or is not a diagonal element
                if (array[row, col] == 0 && row != col)
                {
                    //randomly select path weights from 1 to 20
                    array[row, col] = (byte)random.Next(1, 21);

                    //update percentage filled
                    percent += (1 / (float)array.Length) * 100;
                }
            }

            return array;
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Displays the weights of the paths in canvas
        /// </summary>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x2"></param>
        /// <param name="y2"></param>
        /// <param name="weight"></param>
        private void DisplayPathWeights(float x1, float y1, float x2, float y2, string weight)
        {
            Label label = new Label();
            label.Content = weight;
            label.FontFamily = new FontFamily("Verdana");
            label.FontSize = 15;
            label.VerticalAlignment = VerticalAlignment.Center;
            label.HorizontalAlignment = HorizontalAlignment.Center;
            label.Foreground = Brushes.White;

            //chooses colors based on coordinates, which will be same as the line color
            label.Background = new SolidColorBrush(Color.FromArgb(255, (byte)((x1 + y1 * x1) % 256), (byte)((y1 * x1 + y2 * x2) % 256), (byte)((y2 + x2) % 256)));

            canvasDisplay.Children.Add(label);
            label.SetValue(Canvas.LeftProperty, (double)((x1 + x2) / 2) - 10);
            label.SetValue(Canvas.TopProperty, (double)((y1 + y2) / 2) - 15);
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Dijkstra's single source shortest path algorithm
        /// Returns 2D array: first row is distance from start node and
        /// second row is previous nodes
        /// </summary>
        /// <param name="start"></param>
        /// <param name="graph"></param>
        /// <returns></returns>
        private byte[,] SolveDijkstra(byte start, byte[,] graph)
        {
            bool[] visited = new bool[9];
            byte[] distance = new byte[9];
            byte[] previousNode = new byte[9];

            //initialize all distance(distance from start to the given node) to be infinity(unreachable)
            //initialize all previous nodes to be infinity(unreachable)
            for (int i = 0; i < distance.Length; i++)
            {
                distance[i] = INFINITY;
                previousNode[i] = INFINITY;
            }

            //distance from start node to start node is zero
            distance[start] = 0;

            //add key-value pair of node and distance in priority queue
            PriorityQueue<byte, byte> unvisited = new PriorityQueue<byte, byte>();
            unvisited.Insert(start, 0);

            //iterate until the priority queue is not empty
            while (unvisited.Size != 0)
            {
                //extract the min key-value pair of node-distance
                //node with shortest distance
                PriorityQueue<byte, byte>.KeyValue min = unvisited.ExtractMin();
                byte index = min.Key;
                byte minDistance = min.Value;

                //flag the current node as visited
                visited[index] = true;

                //if the distance is already smaller than minDistance then skip the process
                if (distance[index] < minDistance)
                {
                    continue;
                }

                //iterate through all the neighbors of the current node
                foreach (byte neighbor in GetNeighbors(graph, index))
                {
                    //skip the process if already visited
                    if (visited[neighbor])
                    {
                        continue;
                    }

                    //new distance is the sum of old path plus the weight of the path to the neighbor
                    byte newDistance = ((byte)(distance[index] + graph[index, neighbor]));

                    //proceed only if the new distance is smaller
                    if (newDistance < distance[neighbor])
                    {
                        distance[neighbor] = newDistance;
                        previousNode[neighbor] = index;

                        //insert in priority queue if it is not already there
                        if (!unvisited.Contains(neighbor))
                        {
                            unvisited.Insert(neighbor, newDistance);
                        }
                        //update the distance on priority queue if the node is already in there
                        else
                        {
                            unvisited.UpdateQueue(neighbor, newDistance);
                        }
                    }
                }

            }

            //aggregate distance and previous node in one array           
            byte[,] distance_previous = new byte[2, 9];
            for (int i = 0; i < distance_previous.GetLength(0); i++)
            {
                for (int j = 0; j < distance_previous.GetLength(1); j++)
                {
                    //first row: distance
                    if (i == 0)
                    {
                        distance_previous[i, j] = distance[j];
                    }
                    //second row: previous nodes
                    else if (i == 1)
                    {
                        distance_previous[i, j] = previousNode[j];
                    }
                }
            }

            return distance_previous;
        }

        /// <summary>
        /// Amar Shrestha - 10/5/2020
        /// Dijkstra's algorithm wrapper function.
        /// Traverses the previous nodes to find path from start to end node
        /// Also finds the shortes distance for the path
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="graph"></param>
        /// <returns></returns>
        private List<byte> Dijkstra(byte start, byte end, byte[,] graph)
        {
            //find distance and previous nodes using dijkstra's algorithm
            byte[,] dist_prev = SolveDijkstra(start, graph);

            byte[] distance = new byte[9];
            byte[] previousNodes = new byte[9];
            List<byte> path = new List<byte>();

            //traverse the array to two separate arrays of distance and previous nodes
            //string text = "";
            for (int i = 0; i < dist_prev.GetLength(0); i++)
            {
                for (int j = 0; j < dist_prev.GetLength(1); j++)
                {
                    //first row traversed to distance
                    if (i == 0)
                    {
                        distance[j] = dist_prev[i, j];
                    }
                    //second row traversed to previous nodes
                    else
                    {
                        previousNodes[j] = dist_prev[i, j];
                    }
                }
                //text += "\n";
            }
            //MessageBox.Show(text);

            //return null path if the distance of end node is unreachable (inifinity)
            if (distance[end] == INFINITY) return path;

            //traverse the parent nodes to trace the path
            for (byte i = end; i != INFINITY; i = previousNodes[i])
            {
                path.Add(i);
            }

            path.Reverse();

            //update the shortest path if the path is not empty
            if (path.Count != 0)
            {
                shortestPathLength = distance[end];
            }

            return path;
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Display shortest path of graph in canvas using Dijkstra's algorithm
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnShortestPath_Click(object sender, RoutedEventArgs e)
        {
            path = Dijkstra(0, 8, graph_weighted);

            string text = "";
            foreach (byte node in path)
            {
                text += node + 1 + (node == path[path.Count - 1] ? "" : "->");
            }
            text += "\nPath length: " + (path.Count - 1) + "\n";
            MessageBox.Show(StringFormatted9x9_Array(graph_weighted) + "\n" + text);

            canvasDisplay.Children.Clear();
            DrawGraph(graph_weighted, true, path);
        }

        /// <summary>
        ///  Amar Shrestha - 10/10/2020
        ///  Display the unweighted graph matrix
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnDisplayUnweightedMatrix_Click(object sender, RoutedEventArgs e)
        {
            canvasDisplay.Children.Clear();
            DrawGraph(graph_unweighted);
            MessageBox.Show(StringFormatted9x9_Array(graph_unweighted));
        }

        /// <summary>
        ///  Amar Shrestha - 10/10/2020
        /// Displays the graph with path from start to end node using DFS
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnDFS_Click(object sender, RoutedEventArgs e)
        {
            path.Clear();
            path = DepthFirstSearch(0, 8, graph_unweighted);
            canvasDisplay.Children.Clear();
            DrawGraph(graph_unweighted, false, path);

            string text = "Depth First Search:\n";
            foreach (byte node in path)
            {
                text += node + 1 + (node == path[path.Count - 1] ? "" : "->");
            }
            text += "\nPath length: " + (path.Count - 1) + "\n";
            MessageBox.Show(StringFormatted9x9_Array(graph_unweighted) + text);
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Displays the graph with path from start to end node using BFS
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnBFS_Click_1(object sender, RoutedEventArgs e)
        {
            path.Clear();
            path = BreadthFirstSearch(0, 8, graph_unweighted);
            canvasDisplay.Children.Clear();
            DrawGraph(graph_unweighted, false, path);

            string text = "Breadth First Search:\n";
            foreach (byte node in path)
            {
                text += node + 1 + (node == path[path.Count - 1] ? "" : "->");
            }
            text += "\nPath length: " + (path.Count - 1) + "\n";
            MessageBox.Show(StringFormatted9x9_Array(graph_unweighted) + text);
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Regenerates and displays the unweighted graph matrix
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnRegenerateUnweightedMatrix_Click(object sender, RoutedEventArgs e)
        {
            graph_unweighted = Generate9x9_Matrix();
            canvasDisplay.Children.Clear();
            DrawGraph(graph_unweighted);
            MessageBox.Show(StringFormatted9x9_Array(graph_unweighted));
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Dsiplays the weighted graph matrix
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnDisplayWeightedMatrix_Click(object sender, RoutedEventArgs e)
        {
            canvasDisplay.Children.Clear();
            DrawGraph(graph_weighted, true);
            MessageBox.Show(StringFormatted9x9_Array(graph_weighted));
        }

        /// <summary>
        /// Find and display shortest path in graph using Dijkstra's algorithm
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnShortestPath_Click_1(object sender, RoutedEventArgs e)
        {
            path.Clear();
            path = Dijkstra(0, 8, graph_weighted);
            canvasDisplay.Children.Clear();
            DrawGraph(graph_weighted, true, path);

            string text = "Shortest Path using Dijkstra's algorithm:\n";
            foreach (byte node in path)
            {
                text += node + 1 + (node == path[path.Count - 1] ? "" : "->");
            }
            text += "\nPath length: " + ((shortestPathLength == INFINITY && path.Count == 0) ? "INFINITY" : shortestPathLength.ToString()) + "\n";
            MessageBox.Show(StringFormatted9x9_Array(graph_weighted) + text);
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Regenerates and dispalys weighted graph matrix
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnWeightedMatrix_Click(object sender, RoutedEventArgs e)
        {
            graph_weighted = Generate9x9_WeightedMatrix();
            canvasDisplay.Children.Clear();
            DrawGraph(graph_weighted, true);
            MessageBox.Show(StringFormatted9x9_Array(graph_weighted));
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Draws arrow pointing from one node to another with the same color as the line
        /// </summary>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x2"></param>
        /// <param name="y2"></param>
        /// <param name="radius"></param>
        /// <param name="brush"></param>
        private void DrawPointingArrow(float x2, float y2, float x1, float y1, float radius, SolidColorBrush brush)
        {
            float tip_dist_from_nodeEdge = 0;
            float arrow_length = 60;
            float arrow_width = 20;

            //Tip of the arrow
            float t_tip = (radius + tip_dist_from_nodeEdge) / DistanceFormula(x1, y1, x2, y2);
            float x_tip = ((1 - t_tip) * x1 + t_tip * x2);
            float y_tip = ((1 - t_tip) * y1 + t_tip * y2);

            //base midpoint of arrow triangle
            float t_base = (arrow_length) / DistanceFormula(x1, y1, x2, y2);
            float x_base = ((1 - t_base) * x1 + t_base * x2);
            float y_base = ((1 - t_base) * y1 + t_base * y2);

            //slope of the line
            float m = (y2 - y1) / (x2 - x1);

            //arrow base points
            float x_b1 = (float)(x_base + Math.Sqrt(Math.Pow(arrow_width / 2.0, 2) / (1 + Math.Pow(m, -2))));
            float x_b2 = (float)(x_base - Math.Sqrt(Math.Pow(arrow_width / 2.0, 2) / (1 + Math.Pow(m, -2))));
            float y_b1 = (x_base - x_b1) / m + y_base;
            float y_b2 = (x_base - x_b2) / m + y_base;

            //Use polygon to draw triangle
            Polygon arrow = new Polygon();
            arrow.Stroke = brush;
            arrow.Fill = brush;
            PointCollection points = new PointCollection();
            points.Add(new Point(x_tip, y_tip));
            points.Add(new Point(x_b1, y_b1));
            points.Add(new Point(x_b2, y_b2));
            arrow.Points = points;

            canvasDisplay.Children.Add(arrow);
        }

        /// <summary>
        /// Amar Shrestha - 10/10/2020
        /// Returns the distance between two points
        /// </summary>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x2"></param>
        /// <param name="y2"></param>
        /// <returns></returns>
        private float DistanceFormula(float x1, float y1, float x2, float y2)
        {
            return (float)Math.Sqrt(Math.Pow((x1 - x2), 2) + Math.Pow((y1 - y2), 2));
        }
    }
}
