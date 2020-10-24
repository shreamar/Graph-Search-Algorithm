using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Automation.Peers;

namespace GraphShortestPath
{
    /// <summary>
    /// Amar Shrestha - 9/22/2020
    /// Tree class that has a root node, each node has left and right child node
    /// All child nodes are smaller than their parent node
    /// Data structure is array and only thought view is tree
    /// </summary>
    public class PriorityQueue<T1,T2> where T2 : IComparable<T2>
    {
        #region Private Fields
        private int _EndNodeIndex;
        private T1[] _HeapKey;
        private T2[] _HeapValue;
        #endregion

        #region Constructor

        /// <summary>
        /// Default heap size assigned as 10,000 elements
        /// </summary>
        public PriorityQueue()
        {
            EndNodeIndex = -1;
            _HeapKey = new T1[10000];
            _HeapValue = new T2[10000];
        }

        public PriorityQueue(uint heapSize)
        {
            EndNodeIndex = -1;
            _HeapKey = new T1[heapSize];
            _HeapValue = new T2[heapSize];
        }
        #endregion

        #region Public Properties
        public int EndNodeIndex
        {
            get
            {
                return _EndNodeIndex;
            }
            set
            {
                _EndNodeIndex = value;
            }
        }

        public T1[] HeapKey
        {
            get
            {
                return _HeapKey;
            }
        }

        public T2[] HeapValue
        {
            get
            {
                return _HeapValue;
            }
        }

        public ushort Size
        {
            get
            {
                return (ushort)(EndNodeIndex + 1);
            }
        }
        #endregion

        #region Methods

        /// <summary>
        /// Amar Shrestha - 9/24/2020
        /// Swaps the nodes in the heap array 
        /// </summary>
        /// <param name="nodeA"></param>
        /// <param name="nodeB"></param>
        private void Swap(ref T1 nodeA,ref T1 nodeB)
        {
            T1 temp = nodeA;
            nodeA = nodeB;
            nodeB = temp;
        }

        private void Swap(ref T2 nodeA, ref T2 nodeB)
        {
            T2 temp = nodeA;
            nodeA = nodeB;
            nodeB = temp;
        }

        /// <summary>
        /// Amar Shrestha - 9/22/2020
        /// Inserts data after the end node index then bubbles it up as required
        /// </summary>
        /// <param name="value"></param>
        public void Insert(T1 key, T2 value)
        {
            //insert data after last node index
            EndNodeIndex++;
            _HeapKey[EndNodeIndex] = key;
            _HeapValue[EndNodeIndex] = value;

            //Bubble up until the data is in right node
            BubbleUp(_HeapValue[EndNodeIndex], EndNodeIndex);
        }

        /// <summary>
        /// Amar Shrestha - 9/24/2020
        /// Recursively bubbles up the data in the correct node in the tree
        /// if the data in current node is smaller than the data in parent node,
        /// then it is bubbled up
        /// </summary>
        /// <param name="value"></param>
        /// <param name="nodePosition"></param>
        private void BubbleUp(T2 value, int nodePosition)
        {
            //if the node is the root node, no need to bubble up
            if (nodePosition <= 0)
            {
                return;
            }

            //if the parent is greater than the current node then bubble up
            //direct inequility comparision (<, >, >=, <=) cannot be used on generic type
            if (_HeapValue[(nodePosition - 1) / 2].CompareTo(value) > 0)
            {
                //swap the node with its parents
                Swap(ref _HeapValue[(nodePosition - 1) / 2],ref _HeapValue[nodePosition]);                   //parent of 3 and 4 is 1, so (3-1)/2 = (4-1)/2 = 1
                Swap(ref _HeapKey[(nodePosition - 1) / 2], ref _HeapKey[nodePosition]);

                //recursively bubble up until required
                BubbleUp(value, (nodePosition - 1) / 2);
            }
            else
            {
                return;
            }
        }

        /// <summary>
        /// Amar Shrestha - 9/24/2020
        /// Recursively bubbles down the data in the correct node in the tree.
        /// If the data in current node is greater than the data in child nodes,
        /// then it is bubbled down
        /// </summary>
        /// <param name="value"></param>
        /// <param name="nodePosition"></param>
        private void BubbleDown(T2 value, int nodePosition)
        {
            //When the data reaches the end node bubble down is done
            if (nodePosition >= EndNodeIndex)
            {
                return;
            }

            //(nodePosition * 2) + 1 = Left child node
            //(nodePosition + 1) * 2 = Right child node


            //If the left child is smaller than the current node but right child is not or
            //If both left and right child are smaller than current node but left child is smaller or equal to right child
            //then bubble down to left node
            //Direct inequility comparision (<, >, >=, <=) cannot be used on generic type
            if ((_HeapValue[(nodePosition * 2) + 1].CompareTo(value) < 0 && _HeapValue[(nodePosition + 1) * 2].CompareTo(value) >= 0) ||
                (_HeapValue[(nodePosition * 2) + 1].CompareTo(value) < 0 && _HeapValue[(nodePosition + 1) * 2].CompareTo(value) < 0 &&
                _HeapValue[(nodePosition * 2) + 1].CompareTo(_HeapValue[(nodePosition + 1) * 2]) <= 0))
            {

                //Swap the node with its left child node
                Swap(ref _HeapValue[(nodePosition * 2) + 1], ref _HeapValue[nodePosition]);
                Swap(ref _HeapKey[(nodePosition * 2) + 1], ref _HeapKey[nodePosition]);

                //Recursively bubble down
                BubbleDown(value, (nodePosition * 2) + 1);
            }
            //If the right child is smaller than the current node but left child is not or
            //If both left and right child are smaller than current node but right child is smaller than left child
            //then bubble down to right node
            //Direct inequility comparision (<, >, >=, <=) cannot be used on generic type
            else if ((_HeapValue[(nodePosition * 2) + 1].CompareTo(value) > 0 && _HeapValue[(nodePosition + 1) * 2].CompareTo(value) < 0) ||
                (_HeapValue[(nodePosition * 2) + 1].CompareTo(value) < 0 && _HeapValue[(nodePosition + 1) * 2].CompareTo(value) < 0 &&
                _HeapValue[(nodePosition * 2) + 1].CompareTo(_HeapValue[(nodePosition + 1) * 2]) >= 0))
            {
                //Swap the node with its right child node
                Swap(ref _HeapValue[(nodePosition + 1) * 2], ref _HeapValue[nodePosition]);
                Swap(ref _HeapKey[(nodePosition + 1) * 2], ref _HeapKey[nodePosition]);

                //Recursively bubble down
                BubbleDown(value, (nodePosition + 1) * 2);
            }
        }

        /// <summary>
        /// Amar Shrestha - 9/24/2020
        /// Extracts (pops) the min value from the tree and realigns the tree to maintain heap property 
        /// Swaps the end and root node (min), pops the min value from end node and bubbles down the value
        /// in root node until its in the right node
        /// </summary>
        /// <returns></returns>
        public KeyValue ExtractMin()
        {
            //Swap root node and end node so that min value is in end node
            Swap(ref _HeapKey[0], ref _HeapKey[EndNodeIndex]);
            Swap(ref _HeapValue[0], ref _HeapValue[EndNodeIndex]);

            //store min value in variable and pop it from the tree
            KeyValue min;
            min.Key = _HeapKey[EndNodeIndex];
            min.Value = _HeapValue[EndNodeIndex];
            EndNodeIndex--;

            //Bubble down the new value on the root node until its in the right node
            BubbleDown(_HeapValue[0], 0);

            return min;
        }

        public void UpdateQueue(T1 key, T2 updatedValue)
        {
            ushort nodeIndex = 0;
            for (ushort i = 0; i <= EndNodeIndex; i++)
            {
                if(_HeapKey[i].Equals(key))
                {
                    nodeIndex = i;
                    break;
                }
            }

            if (updatedValue.CompareTo(_HeapValue[nodeIndex]) < 0)
            {
                _HeapValue[nodeIndex] = updatedValue;
                BubbleUp(_HeapValue[nodeIndex], nodeIndex);
            }
            else if(updatedValue.CompareTo(_HeapValue[nodeIndex]) > 0)
            {
                _HeapValue[nodeIndex] = updatedValue;
                BubbleDown(_HeapValue[nodeIndex], nodeIndex);
            }
        }

        public bool Contains(T1 key)
        {
            bool contains = false;
            for (int i = 0; i <= EndNodeIndex; i++)
            {
                if (_HeapKey[i].Equals(key))
                {
                    contains = true;
                    break;
                }
            }

            return contains;
        }

        public struct KeyValue
        {
            public T1 Key;
            public T2 Value;
        }
        #endregion
    }
}
