#! /usr/bin/python2.7
# Zachary Hickton-Jarvis 2021
# Python 2.7.17
# Generate node map from filenames and navigate a path

import rospy
import sys
import os
import re

from vtr_lite.srv import Navigation

#get map directory from parameters
MAPFILETYPE = ".ymal"
with open(sys.path[0][:-3]+"include/param.h", 'r+') as f:
    data = f.read()
matchString = "nh\.param<string>\(\"map_folder\", FOLDER, \"(?P<map_folder>[^\"]+)\""
result = re.search(matchString, data)
try:
    MAPDIR = result.group("map_folder")
except:
    sys.exit("no map_folder specified in param.h")
#get folder for current topological map within map folder
topoMatchString = "nh\.param<string>\(\"topological_map_folder\", TOPOFOLDER, \"(?P<topomap_folder>[^\"]+)\""
result = re.search(topoMatchString, data)
try:
    topoFolder = result.group("topomap_folder")
    if topoFolder[0] == "/": topoFolder = topoFolder[1:]
    TOPOMAPDIR = os.path.join(MAPDIR, topoFolder)
except Exception as e:
    print(e)
    sys.exit("no topological_map_folder specified in param.h")



#load filenames
# files with the format edge__.ymal are accepted 
# with each _ being a node name comprised of a letter followed by any or no combination of numbers
# examples: edgeA1B2.ymal with nodes A1 to B2
#           edgece4.ymal  with nodes c to e4
def loadFilenames():
    try:
        _, _, filenames = next(os.walk(TOPOMAPDIR))
        print("The following files have been found in "+TOPOMAPDIR+":")
        print(filenames)
    except Exception as e:
        print(e)
        sys.exit("Invalid Topological map directory")
        return None
    return filenames
    
def loadVertices(filenames):
    fileMatchstring = re.compile("edge([a-zA-Z]\d*)([a-zA-Z]\d*)\\" + MAPFILETYPE)
    nodes = set()
    vertices = set()
    files = set() #keep the valid filenames to use later
    
    for name in filenames:
        result = re.match(fileMatchstring, name)
        if result:
            node1, node2 = result.group(1), result.group(2)
            nodes.add(node1)
            nodes.add(node2)
            # cost loads the bag's path data distance
            vertices.add(((node1,node2), parseDistance(result.group(0)))) #maintain directionality (node1 -> node2), necessary for knowing when to reverse traversal
            files.add(result.group(0))

    return nodes, vertices, files

def parseDistance(filename):
    #open map file
    try:
        with open(filename, 'r+') as f:
            data = f.read()
        # parse the distance
        distMatchstring = re.compile("map_distance: \[([^]]+)\]")
        result = re.search(distMatchstring, data)
        if result:
            distance = result.group(1)
            return float(distance)
    except:
        return False
        


#build dictionary map
#set of node names, set of vertices form topologicalMap[node][connectedNode][reverse, cost]
def buildMap(nodes, vertices):
    topologicalMap = {}

    #try optimise this
    for node in nodes:
        for vertex in vertices:
            for direction in [0,1]:
                if vertex[0][1-direction] == node:
                    reverse = (direction == 0)
                    if node in topologicalMap:
                        topologicalMap[node][vertex[0][direction]] = {'reverse': reverse, 'cost': vertex[1]}
                    else:
                        topologicalMap[node] = {vertex[0][direction] : {'reverse': reverse, 'cost': vertex[1]}}


    return topologicalMap

#try path planning
def planPath(topologicalMap, nodePath):
    
    #test of bound based breadth first search
    def find_subPath(start, end):
        tempMap = topologicalMap.copy()
        del tempMap[start]
        unseen = tempMap.keys()             #not yet seen nodes
        visible = [(start, 0, 'Start')]     #to be processed nodes
        visited = {}                        #processed nodes
        
        while end not in visited:
            #main loop

            #deque a node
            #sort nodes by cumulative distance here 
            visible = sorted(visible, key=lambda x: x[1])
            visibleNode = visible.pop(0)
            visited[visibleNode[0]] = visibleNode[2]
            #update adjacent nodes
            for node in topologicalMap[visibleNode[0]]:
                if node in unseen:
                    visible.append((node, visibleNode[1] + topologicalMap[visibleNode[0]][node]['cost'], visibleNode[0]))
                    unseen.remove(node)
                
        #get the path
        previous = end
        subPath = [end]
        while previous != "Start":
            subPath.append(visited[previous])
            previous = visited[previous]
        subPath.reverse()
        return subPath[2:]

    path = [nodePath[0]]
    for index in range(len(nodePath)-1):
        #for non nodes (potential instructions) do not add subpath
        try:
            path = path + find_subPath(nodePath[index], nodePath[index+1])
        except:
            sys.exit("No path between " + nodePath[index] + " and " + nodePath[index+1])
    print("-----------------------------------")
    print("Path generated")
    return path


if __name__ == "__main__":
    filenames = loadFilenames()
    nodes, vertices, files = loadVertices(filenames)
    topologicalMap = buildMap(nodes, vertices)
    #verify topological map exists
    if (topologicalMap.keys() == []): sys.exit("No valid files in " + TOPOMAPDIR)
    print("\nThe following nodes have been detected:")
    print(topologicalMap.keys()) #Check using the map as the nodes in the variable 'nodes' may not be valid
    walk = raw_input("\nPlease enter the nodes you want to visit in order, separated by spaces:\n").split(" ")
    #walk = ['D', 'A', 'C', 'B', 'D']
    
    #verify the walk
    validWalk = True
    prevNode = " "
    if (walk == [''] or walk is None):
        sys.exit("Please enter a sequence of nodes")
    else:
        for node in walk:
            if (prevNode == node or not (node in topologicalMap.keys())):
                validWalk = False
            prevNode = node
        if (not validWalk):
            sys.exit("Invalid sequence of nodes")

    #plan the path
    path = planPath(topologicalMap, walk)

    #navigate each subpath
    for index in range(len(path)-1):
        #make this function for correct direction
        firstNode = path[index] 
        secondNode = path[index+1] 

        #account for reverse navigating an edge
        if (topologicalMap[firstNode][secondNode]['reverse']):
            edgeName = "edge" + path[index+1] + path[index]
            reverse = "true"
        else:
            edgeName = "edge" + path[index] + path[index+1]
            reverse = "false"
        edgeFile = edgeName + MAPFILETYPE
        
        if (edgeFile in files):
            raw_input("\nNavigating "+edgeName+", reverse: "+reverse+" \nPress enter to continue...\n")
            # all topological map folders should be within the map directory, so that they can be called by navigator

            rospy.wait_for_service('vtr_lite/navigator')
            try:
                navigatorCall = rospy.ServiceProxy('vtr_lite/navigator', Navigation)
                resp1 = navigatorCall(edgeFile,reverse)
                print(resp1.status)
                #wait for end of naviagtion
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
    sys.exit("Navigation finished")