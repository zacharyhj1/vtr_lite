#! /usr/bin/python2.7
# Zachary Hickton-Jarvis 2021
# Python 2.7.17
# Generate node map from filenames and navigate a path
# To run a simulation: ensure you have reversed playback versions of your rosbags used in simulation
# they must be named the same as the original rosbag with 'reversed' before '.bag'
# add the '--simulation' flag to the script call to run a simulation
# add the '--step' flag to the step through each navigator call

import rospy
import sys
import os
import re
import time 
import subprocess
import argparse

from std_msgs.msg import Float32
from vtr_lite.srv import Navigation


rospy.init_node('topological_nav')
#get map directory from parameters
MAPFILETYPE = ".yaml"
try:
    matchString = "nh\.param<string>\(\"map_folder\", FOLDER, \"(?P<map_folder>[^\"]+)\""
    with open(sys.path[0][:-3]+"include/param.h", 'r+') as f:
        data = f.read()
    result = re.search(matchString, data)
    MAPDIR = result.group("map_folder")
except Exception as e:
    print(e)
    sys.exit("no map_folder specified in param.h")
#get folder for current topological map within map folder
try:
    topoMatchString = "nh\.param<string>\(\"topological_map_folder\", TOPOFOLDER, \"(?P<topomap_folder>[^\"]+)\""
    result = re.search(topoMatchString, data)
    TOPOMAPNAME = result.group("topomap_folder")
    if TOPOMAPNAME[0] == "/": TOPOMAPNAME = TOPOMAPNAME[1:]
    TOPOMAPDIR = os.path.join(MAPDIR, TOPOMAPNAME)
except Exception as e:
    print(e)
    sys.exit("invalid topological_map_folder specified in param.h")



#load filenames
# files with the format edge__.yaml are accepted 
# with each _ being a node name comprised of a letter followed by any or no combination of numbers
# examples: edgeA1B2.yaml with nodes A1 to B2
#           edgece4.yaml  with nodes c to e4
def loadFilenames():
    '''
    Retrieves the filenames of every file in the directory specified by MAP_FOLDER and TOPOLOGICAL_MAP_FOLDER in params.h
    Returns
    ----------
    filenames : List
        A list of filenames from the folder specified in include/params.h
    '''
    print(TOPOMAPDIR)
    try:
        _, _, filenames = next(os.walk(TOPOMAPDIR))
        print("The following files have been found in "+TOPOMAPDIR+":")
        print(filenames)
    except Exception as e:
        sys.exit("Invalid Topological map directory %s" %e)
    return filenames
    
def loadEdges(filenames):
    '''
    Generates the set of valid edges and nodes of the topological map from a list of filenames
    Filenames with the format edge__.yaml are accepted 
    with each _ being a node name comprised of a letter followed by any or no combination of numbers
    examples: edgeA1B2.yaml with nodes A1 to B2
              edgece4.yaml  with nodes c to e4
    Parameters
    ----------
    filenames : List
        A list of strings containing filenames
    Returns
    ----------
    nodes : Set
        The set of all valid nodes detected from the filenames
    edges : Set
        A set of tuples containing the node pair and distance of each valid edge
    files : Set
        The set of all valid filenames for use in the topological map
    '''
    
    try:
        fileMatchstring = re.compile("edge([a-zA-Z]\d*)([a-zA-Z]\d*)\\" + MAPFILETYPE)
        nodes = set()
        edges = set()
        files = set() #keep the valid filenames to use later
        
        for name in filenames:
            result = re.match(fileMatchstring, name)
            if result:
                node1, node2 = result.group(1), result.group(2)
                nodes.add(node1)
                nodes.add(node2)
                # cost loads the bag's path data distance
                edges.add(((node1,node2), parseDistance(result.group(0)))) 
                #maintains directionality (node1 -> node2), necessary for knowing when to reverse traversal
                files.add(result.group(0))
    except Exception as e:
        sys.exit("Invalid Topological map directory %s" %e)
    return nodes, edges, files

def parseDistance(filename):
    '''
    Parses the text of map files to retrieve the distance of the map
    Parameters
    ----------
    filename : String
        The filename of the map file
    Returns
    ----------
    distance : Float
    The distance of the navigation recorded in the map file
    '''

    #open map file
    filename = os.path.join(TOPOMAPDIR,filename)
    try:
        with open(filename, 'r+') as f:
            data = f.read()
        # parse the distance
        distMatchstring = re.compile("map_distance: \[([^]]+)\]")
        result = re.search(distMatchstring, data)
        if result:
            distance = result.group(1)
            return float(distance)
    except Exception as e:
        print("No map_distance found in %s %s" %(filename,e))
        return False
        


def buildMap(nodes, edges):
    '''
    Builds a topological map from node names and edges
    Parameters
    ----------
    nodes : List
        A list of node names in String format
    edges : List
        A list of edges in Tuple format
    Returns
    ----------
    topologicalMap : Dictionary
    A multilevel dictionary representing a high-level topological map, accessible with: topologicalMap[node][connectedNode][reverse, cost]
    '''

    topologicalMap = {}

    #try optimise this
    for node in nodes:
        for edge in edges:
            for direction in [0,1]:
                if edge[0][1-direction] == node:
                    reverse = (direction == 0)
                    if node in topologicalMap:
                        #do not overwrite existing non-reverse connections
                        if (not edge[0][direction] in topologicalMap[node]) or topologicalMap[node][edge[0][direction]]['reverse'] == True:
                            topologicalMap[node][edge[0][direction]] = {'reverse': reverse, 'cost': edge[1]}
                    else:
                        topologicalMap[node] = {edge[0][direction] : {'reverse': reverse, 'cost': edge[1]}}


    return topologicalMap


def planPath(topologicalMap, nodePath):
    '''
    Plan a path between all nodes in the nodePath using the map supplied in topologicalMap
    Parameters
    ----------
    topologicalMap : Dictionary
        A multi-level Dictionary representing a high-level topological map.
    nodePath : List
        A List with a start node at index [0] and a sequence of destination nodes after.
    Returns
    ----------
    path : List
        A list of nodes comprised of shortest-path routes between the nodes in nodePath
    pathDistance : float
        The total distance of the path generated
    '''
    
    def find_subPath(start, end):
        '''
        A Dijkstra's based shortest-path algorithm
        Parameters
        ----------
        start : String
            The name of the starting node
        end : String
            The name of the destination node
        Returns
        ----------
        subPath[:2] : List
            A list of nodes comprised of the shortest path route betwen start and end, without the start node 
        distance : float
            The total distance of the path generated
        '''
        tempMap = topologicalMap.copy()
        del tempMap[start]
        unseen = tempMap.keys()             # not yet seen nodes
        visible = [(start, 0, 'Start')]     # to be processed nodes
        visited = {}                        # processed nodes
        distance = 0
        
        while end not in visited:
            #deque a node
            #sort nodes by cumulative distance here 
            visible = sorted(visible, key=lambda x: x[1])
            visibleNode = visible.pop(0)
            visited[visibleNode[0]] = visibleNode[2]
            #keeps track of chosen path's distance
            if visibleNode[1]> distance: distance = visibleNode[1]
            #update adjacent nodes
            for node in topologicalMap[visibleNode[0]]:
                if node in unseen:
                    try:
                        visible.append((node, visibleNode[1] + topologicalMap[visibleNode[0]][node]['cost'], visibleNode[0]))
                    except Exception as e:
                        print("%s \nWarning: edge %s to %s has no distance value" %(e,visiblenode[0],node))
                    unseen.remove(node)
                #if lower cost for node found, update visible
                else:
                    visibleKeys = [visNode[0] for visNode in visible]
                    if (node in visibleKeys):
                        nodePos = visibleKeys.index(node)
                        currentCost = visible[nodePos][1]
                        potentialCost = visibleNode[1] + topologicalMap[visibleNode[0]][node]['cost']
                        if (potentialCost < currentCost):
                            visible[nodePos] = (node, potentialCost, visibleNode[0])


                
        #get the path
        previous = end
        subPath = [end]
        while previous != "Start":
            subPath.append(visited[previous])
            previous = visited[previous]
        
        subPath.reverse()
        return subPath[2:], distance

    path = [nodePath[0]]
    pathDistance = 0
    for index in range(len(nodePath)-1):
        try:
            subPath, subPathDistance = find_subPath(nodePath[index], nodePath[index+1])
            path = path + subPath
            pathDistance += subPathDistance
        except Exception as e:
            print("No path between " + nodePath[index] + " and " + nodePath[index+1])
            sys.exit(e)
    print("-----------------------------------\nPath generated")
    return path, pathDistance


def main():
    #parse argument
    parser = argparse.ArgumentParser(description='Check for --simulation flag and --step flag')
    parser.add_argument('--simulation', dest='simulation', action='store_true')
    parser.add_argument('--step', dest='step', action='store_true')
    parser.set_defaults(simulation = False, step = False)
    args = parser.parse_args()

    #load filenames in specified directory, build map
    filenames = loadFilenames()
    nodes, vertices, files = loadEdges(filenames)
    topologicalMap = buildMap(nodes, vertices)
    print(topologicalMap)
    #verify topological map exists
    if (topologicalMap.keys() == []): sys.exit("No valid files in " + TOPOMAPDIR)
    print("\nThe following nodes have been detected:")
    print(topologicalMap.keys()) #Check using the map as the nodes in the variable 'nodes' may not be valid
    try:
        walk = raw_input("\nPlease enter the nodes you want to visit in order, separated by spaces:\n").strip().split(" ")
    except KeyboardInterrupt as e:
        sys.exit("\nNavigation cancelled by user")
    
    #verify the walk
    prevNode = " "
    if (walk == [''] or walk is None):
        sys.exit("Please enter a sequence of nodes")
    else:
        for node in walk:
            if (prevNode == node or not (node in topologicalMap.keys())):
                sys.exit("Invalid sequence of nodes")
            prevNode = node


    #plan the path
    path, pathDistance = planPath(topologicalMap, walk)
    print(path)
    
    #navigate each subpath
    currentDistance = 0
    for index in range(len(path)-1):
        #make this function for correct direction
        firstNode = path[index] 
        secondNode = path[index+1] 
        subPathDistance = topologicalMap[firstNode][secondNode]['cost']
        print("Distance progress %s/%s" %(currentDistance, pathDistance))

        #account for reverse navigating an edge
        if (topologicalMap[firstNode][secondNode]['reverse']):
            edgeName = "edge" + path[index+1] + path[index]
            reverse = True
        else:
            edgeName = "edge" + path[index] + path[index+1]
            reverse = False
        edgeFile = os.path.join(TOPOMAPNAME, edgeName)
        
        #wait for user input if --step
        if ((edgeName+MAPFILETYPE) in files):
            if (not args.step):
                if reverse:
                    print("\nNavigating "+edgeName+", reverse: true\n")
                else:
                    print("\nNavigating "+edgeName+", reverse: false\n")
            else:
                if reverse:
                    raw_input("\nNavigating "+edgeName+", reverse: true \nPress enter to start navigation...\n")
                else:
                    raw_input("\nNavigating "+edgeName+", reverse: false \nPress enter to start navigation...\n")

            # all topological map folders should be within the map directory, so that they can be called by navigator
            print("Navigating...")

            rospy.wait_for_service('vtr_lite/navigator')
            try:
                #play rosbag if --simulation
                if args.simulation:
                    
                    if reverse: 
                        bagfile = os.path.join(MAPDIR,edgeFile + "reversed.bag")
                    else:
                        bagfile = os.path.join(MAPDIR,edgeFile + ".bag")
                    rosbag_play = subprocess.Popen(['rosbag', 'play', bagfile],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
                    print("Playing rosbag %s" %bagfile)


                #call navigator
                print("Calling navigator")
                navigatorCall = rospy.ServiceProxy('vtr_lite/navigator', Navigation, persistent=True)
                res1 = navigatorCall(edgeFile,reverse)
                while True:
                    if res1.status:
                        break
                print("Closing navigator")
                navigatorCall.close()

                currentDistance += subPathDistance
            except rospy.ServiceException as e:
                sys.exit("Service call failed: %s"%e)
            finally:
                #close call/rosbag if error occurs
                navigatorCall.close()
                if args.simulation and (rosbag_play.poll is None):
                    print("Closing Rosbag")
                    rosbag_play.send_signal(subprocess.signal.SIGINT)
                    rosbag_play.terminate()
        else:
            sys.exit("Cannot locate edge in topological map file directory")
    print("Distance progress %s/%s" %(currentDistance, pathDistance))
    sys.exit("Navigation finished")

if __name__ == "__main__":
    main()
