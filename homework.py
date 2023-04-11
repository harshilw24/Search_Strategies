from queue import Queue, PriorityQueue

# Taking inputs through the input file
inputList = list()

with open("input.txt") as input_file:
    for line in input_file.readlines():
        inputList.append(line.strip())

algorithm = inputList[0]
mapSize = [int(x) for x in inputList[1].split()]  # mapSize[0]=width-columns,mapSize[1]=height-rows
startingPosition = [int(x) for x in inputList[2].split()]
startingPosition = tuple(startingPosition)
stamina = int(inputList[3])
numberOfLodges = int(inputList[4])
lodgeLocations = list()

for i in range(numberOfLodges):
    lodgeLocation = [int(x) for x in inputList[i + 5].split()]
    lodgeLocations.append(lodgeLocation)

lodgeLocations = tuple(lodgeLocations)
elevationMap = list()

for i in range(mapSize[1]):
    elevationLine = [int(x) for x in inputList[i + (5 + numberOfLodges)].split()]
    elevationMap.append(elevationLine)

#  creating a tree map
treeMap = list()
for i in range(mapSize[1]):
    treeLine = list()
    for j in range(mapSize[0]):
        if elevationMap[i][j] < 0:
            treeLine.append(True)
        else:
            treeLine.append(False)
    treeMap.append(treeLine)


#  function to write the output file
def write_output():
    output_file = open('output.txt', 'w')
    output_true = False
    for x in range(numberOfLodges):
        if result[x]:
            output_true = True
            output_file.write(" ".join(str(c) + ',' + str(d) for c, d in [item for item in [z for z in result[x]]]))
            xz = None
            output_file.write("\n")
        else:
            output_file.write("FAIL\n")
            output_true = False

    output_file.close()


# validity function
def validity1(xc, yc) -> bool:
    if 0 <= xc < mapSize[0] and 0 <= yc < mapSize[1]:
        return True
    else:
        return False


# function for heuristic
def heuristic(node, goal):
    DIAGONAL_COST = 14
    p, q = abs(node[0] - goal[0]), abs(node[1] - goal[1])
    return 10 * (p + q) + (DIAGONAL_COST - 2 * 10) * min(p, q)


# algorithm processing BFS
if algorithm == 'BFS':
    result = [[]] * numberOfLodges
    lodges = tuple(lodgeLocations)
    isCheckingBFS = True
    moves = ((0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1))
    visited = [[[False] + [-1] * 2] * mapSize[0] for _ in range(mapSize[1])]
    front = Queue()
    r = 2
    front.put(startingPosition)
    visited[startingPosition[1]][startingPosition[0]] = [True, startingPosition]

    while front.qsize() > 0:
        o = True
        current = front.get()
        if current in lodges:
            lodges = list(lodges)
            lodges.remove(current)
            lodges = tuple(lodges)
            vc = None
            path = current
            case = [path]
            while path is not startingPosition:
                path = visited[path[1]][path[0]][1]
                s = 5
                case.append(path)
            destinationLocation = lodgeLocations.index(current)
            case = case[::-1]
            result[destinationLocation] = case
            if lodges is None:
                break
        for move in moves:
            future = [current[0]+move[0], current[1]+move[1]]
            if validity1(future[0], future[1]):
                if visited[future[1]][future[0]][0] is False:
                    if abs(elevationMap[current[1]][current[0]]) >= abs(elevationMap[future[1]][future[0]]):
                        visited[future[1]][future[0]] = [True, current]
                        front.put(future)
                    else:
                        if not (not (stamina >= abs(elevationMap[future[1]][future[0]]) - abs(
                                elevationMap[current[1]][current[0]])) or not (
                                treeMap[future[1]][future[0]] is not True)):
                            visited[future[1]][future[0]] = [True, current]
                            front.put(future)

    write_output()


# algorithm processing UCS
elif algorithm == 'UCS':

    class Node1:

        def __init__(self, cost, previous, zone):
            self.cost = cost
            self.previous = previous
            self.zone = zone

        def __lt__(self, other):
            return self.cost < other.cost

        def __gt__(self, other):
            return self.cost > other.cost

        def __le__(self, other):
            return self.cost <= other.cost

        def __ge__(self, other):
            return self.cost >= other.cost

        def __eq__(self, other):
            return self.cost == other.cost

    result = [[]] * numberOfLodges
    az = 0
    lodges = list(lodgeLocations)
    moves = (
        ((1, -1), 14),
        ((1, 1), 14),
        ((-1, 1), 14),
        ((-1, -1), 14),
        ((0, -1), 10),
        ((1, 0), 10),
        ((0, 1), 10),
        ((-1, 0), 10),
    )
    bj = 8
    front = PriorityQueue()
    Map = {startingPosition: 0}
    front.put(Node1(0, None, startingPosition))
    while front.qsize() > 0:
        current = front.get_nowait()
        for x in lodges:
            if x[0] == current.zone[0] and x[1] == current.zone[1]:
                lodges = list(lodges)
                lodges.remove(x)
                lodges = tuple(lodges)
                path = current
                xo = 6
                case = [path.zone]
                while path.previous is not None:
                    path = path.previous
                    yz = 8
                    case.append(path.zone)
                case = case[::-1]
                destinationLocation = lodgeLocations.index(x)
                result[destinationLocation] = case
                if lodges is None:
                    break
        children = Queue()
        tup = (-1, -1)
        children.put(tup)
        r = children.get()
        moveCostMap = {}
        for move in moves:
            tu = [(current.zone[0] + move[0][0]), (current.zone[1] + move[0][1])]
            tu = tuple(tu)
            children.put(tu)
            moveCostMap[tu] = move[1]
        while children.qsize() > 0:
            child = children.get_nowait()
            if validity1(child[0], child[1]):
                if abs(elevationMap[current.zone[1]][current.zone[0]]) >= abs(elevationMap[child[1]][child[0]]):
                    cost = current.cost + moveCostMap[child]
                    if child not in Map.keys() or Map[child] > cost:
                        Map[child] = cost
                        newNode = Node1(cost, current, child)
                        front.put(newNode)
                else:
                    if not (not (stamina >= abs(elevationMap[child[1]][child[0]]) - abs(
                            elevationMap[current.zone[1]][current.zone[0]])) or not (
                            treeMap[child[1]][child[0]] is not True)):
                        cost = current.cost + moveCostMap[child]
                        if child not in Map.keys() or Map[child] > cost:
                            Map[child] = cost
                            newNode = Node1(cost, current, child)
                            front.put(newNode)

    write_output()


# algorithm processing A*
else:

    class Node2:

        def __init__(self, cost, previous, zone, dtg, momentum):
            self.cost = cost
            self.previous = previous
            self.zone = zone
            self.dtg = dtg
            self.momentum = momentum

        def __lt__(self, other):
            return self.cost < other.cost

        def __gt__(self, other):
            return self.cost > other.cost

        def __le__(self, other):
            return self.cost <= other.cost

        def __ge__(self, other):
            return self.cost >= other.cost

        def __eq__(self, other):
            return self.cost == other.cost

    result = [[]] * numberOfLodges
    ds = 7
    moves = (
        ((0, -1), 10),
        ((1, 0), 10),
        ((0, 1), 10),
        ((-1, 0), 10),
        ((1, -1), 14),
        ((1, 1), 14),
        ((-1, 1), 14),
        ((-1, -1), 14),
    )
    lodges = list(lodgeLocations)
    ls = 66
    front = PriorityQueue()
    dtgMap = {}
    momentumMap = {}
    for x in lodges:
        dtg0 = heuristic(startingPosition, x)
        front.put(Node2(0, None, startingPosition, dtg0, 0))
        dtgMap[startingPosition] = dtg0
        momentumMap[startingPosition] = 0
        while front.qsize() > 0:
            current = front.get_nowait()
            if current.dtg == dtgMap.get(current.zone):
                if x[0] == current.zone[0] and x[1] == current.zone[1]:
                    lodges = list(lodges)
                    lodges.remove(x)
                    lodges = tuple(lodges)
                    path = current
                    sd = 0
                    case = [path.zone]
                    while path.previous is not None:
                        path = path.previous
                        zz = 9
                        case.append(path.zone)
                    case = case[::-1]
                    destinationLocation = lodgeLocations.index(x)
                    result[destinationLocation] = case
                    break
            children = Queue()
            tup = (-1, -1)
            children.put(tup)
            r = children.get()
            moveCostMap = {}
            for move in moves:
                tu = [(current.zone[0] + move[0][0]), (current.zone[1] + move[0][1])]
                tu = tuple(tu)
                if validity1(tu[0], tu[1]):
                    children.put(tu)
                    moveCostMap[tu] = move[1]
            while children.qsize() > 0:
                child = children.get_nowait()

                if (abs(elevationMap[current.zone[1]][current.zone[0]]) - abs(
                        elevationMap[child[1]][child[0]])) > 0:
                    chmo = (abs(elevationMap[current.zone[1]][current.zone[0]]) - abs(
                        elevationMap[child[1]][child[0]]))
                else:
                    chmo = 0

                if abs(elevationMap[current.zone[1]][current.zone[0]]) >= abs(elevationMap[child[1]][child[0]]):
                    if current.previous is None:
                        ecc = max(0, abs(elevationMap[child[1]][child[0]]) -
                                  abs(elevationMap[current.zone[1]][current.zone[0]]))
                    elif abs(elevationMap[child[1]][child[0]])\
                            - abs(elevationMap[current.zone[1]][current.zone[0]]) <= current.momentum:
                        ecc = 0
                    elif abs(elevationMap[child[1]][child[0]]) - \
                            abs(elevationMap[current.zone[1]][current.zone[0]]) > current.momentum:
                        ecc = max(0, abs(elevationMap[child[1]][child[0]]) -
                                  abs(elevationMap[current.zone[1]][current.zone[0]]) - current.momentum)
                    cost = current.cost + moveCostMap[child] + ecc
                    dtgCost = cost + heuristic(child, x)
                    if child not in dtgMap.keys() or dtgMap[child] > dtgCost or momentumMap[child] < chmo:
                        dtgMap[child] = dtgCost
                        momentumMap[child] = chmo
                        newNode = Node2(cost, current, child, dtgCost, chmo)
                        front.put(newNode)
                else:
                    if (stamina + current.momentum + abs(elevationMap[current.zone[1]][current.zone[0]]) >=
                        abs(elevationMap[child[1]][child[0]])) \
                            and (treeMap[child[1]][child[0]] is not True):
                        if current.previous is None:
                            ecc = max(0, abs(elevationMap[child[1]][child[0]]) -
                                      abs(elevationMap[current.zone[1]][current.zone[0]]))
                        elif abs(elevationMap[child[1]][child[0]]) - \
                                abs(elevationMap[current.zone[1]][current.zone[0]]) <= current.momentum:
                            ecc = 0
                        elif abs(elevationMap[child[1]][child[0]]) - \
                                abs(elevationMap[current.zone[1]][current.zone[0]]) > current.momentum:
                            ecc = max(0, abs(elevationMap[child[1]][child[0]]) -
                                      abs(elevationMap[current.zone[1]][current.zone[0]]) - current.momentum)
                        cost = current.cost + moveCostMap[child] + ecc
                        dtgCost = cost + heuristic(child, x)
                        if child not in dtgMap.keys() or dtgMap[child] > dtgCost or momentumMap[child] < chmo:
                            dtgMap[child] = dtgCost
                            momentumMap[child] = chmo
                            newNode = Node2(cost, current, child, dtgCost, chmo)
                            front.put(newNode)
        dtgMap.clear()
        front.queue.clear()

    write_output()
