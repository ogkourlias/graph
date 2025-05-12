#!/usr/bin/env python3

"""
    usage:
        python3 algorithms
"""

# METADATA VARIABLES
__author__ = "Orfeas Gkourlias"
__status__ = "WIP"
__version__ = "0.1"

# IMPORTS
import sys
from queue import Queue
import networkx as nx

# FUNCTIONS
def kahn(network):
    # Creating starting lists and queue
    sorted = []
    node_watch = [] # To watch whether node is in the queue yet (messy solution, not sure how to check the queue itself atm)
    queue = Queue()

    # Put all nodes with incoming degree of 0 into the queue and node watch
    for node in network.in_degree:
        if node[1] == 0:
            queue.put(node)
            node_watch.append(node[0])

    # While there are still nodes in the queue
    while not queue.empty():
        # Get first node in the queue
        curr_node = queue.get()
        # Node should have no incoming edges, therefore add it to the new sorted list
        sorted.append(curr_node[0])
        # Remove it from the network
        network.remove_node(curr_node[0])  
        
        # Check after node removal whether there are any nodes not already in the queue which now have in degree of 0
        for node in network.in_degree:
            if node[1] == 0 and node[0] not in node_watch:
                queue.put(node)
                node_watch.append(node[0])

    return sorted

def bfs(network, root):
    # Create intial queue and lists
    queue = Queue()
    root_node = [node for node in network.nodes if node == root][0]
    queue.put(root_node)
    
    # Logging the search order
    search_order = [root_node]
    
    # Node watch to track which nodes are already queued for search
    node_watch = [root_node]
    
    # While there is work to do
    while not queue.empty():
        curr_node = queue.get()

        # Check neighbors of node
        for neighbor in nx.neighbors(network, curr_node):

            # Check if neighbor is not already queued by another node
            if neighbor not in node_watch:
                node_watch.append(neighbor)
                queue.put(neighbor)
                search_order.append(neighbor)

    return search_order

# MAIN
def main(args):
    """Main function"""
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))