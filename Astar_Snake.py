
import random
from tkinter import *
from tkinter import ttk
from nbformat import current_nbformat
from pynput import keyboard
import time
import numpy as np

direction = 0

path = []


def heuiristic(pos, goal, width):
    x = pos % width
    y = (pos - x)/width

    goalX = goal % width
    goalY = (goal - goalX)/width

    diff = abs(goalX-x) + abs(goalY-y)
    return diff


def printGrid(grid, width):
    row = ""
    for i in range(0, len(grid)):
        if i % width == 0:
            print(row)
            row = ""
        row += str(grid[i]) + " "
    return None


def myReconstructPath(current, distanceFromStart, start, grid, width):
    current = current
    finalPath = []
    while current != start:
        neighbors = findNeighbor(current, grid, width)
        shortestNeighborDistance = float("inf")
        shortestNeighbor = None
        for neighbor in neighbors:
            if distanceFromStart[neighbor] < shortestNeighborDistance:
                shortestNeighborDistance = distanceFromStart[neighbor]
                shortestNeighbor = neighbor

        finalPath.append(current)
        current = shortestNeighbor
    finalPath.append(start)
    return finalPath


def reconstructPath(current, camefrom, start):
    totalPath = [current]
    while camefrom[current] != float("inf"):
        current = camefrom[current]
        totalPath.append(current)
    return totalPath


def findNeighbor(pos, grid, width):
    neighbors = []
    if (0 <= pos - width) and grid[pos-width] != 9:
        neighbors.append(pos - width)
    if ((0 <= pos - 1) and pos % width > (pos-1) % width) and grid[pos-1] != 9:
        neighbors.append(pos - 1)
    if (0 <= pos + 1 and pos % width < (pos+1) % width) and grid[pos+1] != 9:
        neighbors.append(pos + 1)
    if (0 <= pos + width) and pos + width < len(grid) and grid[pos+width] != 9:
        neighbors.append(pos + width)
    return neighbors


def aStar(start, goal, grid, width, h=heuiristic):
    #    My implementation of wiki psudocode
    visited = []
    openSet = [int(start)]

    cameForm = []
    for i in range(0, len(grid)):
        cameForm.append(float("inf"))

    gScore = []
    for i in range(0, len(grid)):
        gScore.append(float("inf"))
    gScore[int(start)] = 0

    fScore = []
    for i in range(0, len(grid)):
        fScore.append(float("inf"))
    fScore[int(start)] = h(start, goal, width)
    # print("fScore:", h(start, goal, width))
    iteration = 0
    while len(openSet) > 0:
        iteration += 1
        if iteration > len(grid) ^ 2:
            return 0

        current = 0
        bestChoice = float("inf")
        for idx, i in enumerate(fScore):
            if i < bestChoice and idx not in visited:
                bestChoice = i
                current = idx
        visited.append(current)

        # print("openSet", openSet)
        # print("current", current)
        if current == goal:
            return reconstructPath(current, cameForm, start)

        if current in openSet:
            openSet.remove(current)

        neighbors = findNeighbor(current, grid, width)

        for neighbor in neighbors:
            tenative_gScore = gScore[current] + grid[neighbor]
            if tenative_gScore < gScore[neighbor]:
                cameForm[neighbor] = current
                gScore[neighbor] = tenative_gScore
                fScore[neighbor] = tenative_gScore + h(current, goal, width)
                if neighbor not in openSet:
                    openSet.append(neighbor)

    return 0


def snakeMove(current, dir, width):
    # Uses a direction to calculate the next oisition of snake head
    if dir == 0:
        if current-width < 0:
            return "illegal move"
        return current-width
    elif dir == 1:
        if current+width > width*width:
            return "illegal move"
        return current+width
    elif dir == 2:
        if (current) % width < (current-1) % width:
            return "illegal move"
        return current-1
    elif dir == 3:
        if (current) % width > (current+1) % width:
            return "illegal move"
        return current+1


def generateMap(width):
    # Generates a one dimentional map where each value is 0
    snakeMap = []
    for i in range(0, width * width):
        snakeMap.append(1)

    return snakeMap


def mapToCoord(pos, width):
    # based on map index and width calculates the x and y value of the position
    x = pos % width
    y = (pos - x)/width
    return x, y


def on_press(key):
    # changes direction when an arowkey is pressed down
    global direction
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
    except AttributeError:
        if(key == keyboard.Key.up):
            direction = 0
        elif (key == keyboard.Key.down):
            direction = 1
        elif (key == keyboard.Key.left):
            direction = 2
        elif (key == keyboard.Key.right):
            direction = 3


def panicMode(origin, grid, width, snake):
    moves = []
    neighbors = findNeighbor(origin, grid, width)
    if origin-1 in neighbors:
        moves.append(origin-1)
    if origin-width in neighbors:
        moves.append(origin-width)
    if origin+1 in neighbors:
        moves.append(origin+1)
    if origin+width in neighbors:
        moves.append(origin+width)

    place_holder = None
    best_move_rating = 0
    best_move = 0
    for idx, move in enumerate(moves):
        current = 0
        current, place_holder = floodSearch(move, grid, width, 10, snake)
        if current > best_move_rating:
            best_move_rating = current
            best_move = idx
    if len(moves) > 0:
        return moves[best_move]
    else:
        return origin


def floodSearch(origin, grid, width, target, snake):
    openSet = [origin]
    visited = set()
    targetFound = False
    while len(openSet) > 0:
        current = openSet.pop()
        # visited.append(current)
        visited.add(current)
        if grid[int(current)] == target:
            targetFound = True
        for neighbor in findNeighbor(int(current), grid, width):
            if neighbor not in visited and grid[neighbor] != 9:
                openSet.append(neighbor)
    free_squares_covered = len(visited)/(len(grid)-len(snake))
    return free_squares_covered, targetFound


def AstarMove(map, gridwidth, headPos, snake, path, applePos):
    temp_map = []

    for i in map:
        temp_map.append(i)

    for i in snake:
        temp_map[int(i)] = 9
    temp_map[int(headPos)] = 9

    area_covered, target_found = floodSearch(
        headPos, temp_map, gridwidth, 4, snake)

    if len(path) > 0:
        best_move = path.pop()
    else:
        path = aStar(headPos, applePos, temp_map, gridwidth)
        if path == 0:
            best_move = panicMode(headPos, temp_map, width, snake)
            path = []
        else:
            path.pop()
            best_move = path.pop()
    return best_move, path


def visualize(widowWidth, grid, gridWidth, visualize=bool, network=0, vizAI=False):
    # main function running the game

    # the width of each grid square
    squareWidth = widowWidth/gridWidth

    # the position of the snake head is the middle of the board
    headPos = gridWidth*gridWidth/2+gridWidth/2

    # if train:
    #    direction = 0
    #    map = generateMap(width)
    #
    global direction

    # initiates canvas with white backgroud

    if visualize or vizAI:
        master = Tk()
        C = Canvas(master, bg="white", height=widowWidth, width=widowWidth)
        C.pack()

    # sets apples' position to random index on map
    applePos = random.randint(0, len(map)-1)
    map[applePos] = 4

    score = 0
    path = []
    gameLoop = True

    headPos = (width*width/2+width/2)

    snake = []

    # runs loop while listening to keypresses
    if visualize:
        with keyboard.Listener(on_press=on_press) as listener:
            while gameLoop:
                prevHeadPos = headPos
                gameLoop = True

                #AIInput = WatchDirection()

                # moves the head acording to direction
                headPos = snakeMove(headPos, direction, gridWidth)

                # if snakeMove returns "illegal move" the game ends
                if headPos == "illegal move":
                    gameLoop = False
                    return score

                if headPos in snake or headPos == prevHeadPos:
                    gameLoop = False
                    return score

                # if the head is not on the apple the last grid on snake dissaperes, otherwhise it remains
                if headPos != applePos:
                    if len(snake) > 0:
                        snake.pop(0)
                    snake.append(prevHeadPos)
                else:
                    snake.append(prevHeadPos)
                    score += 1

                # when the head is eaten a new one will appere in a random position,
                # if its appers on the snake a new position is generated
                if applePos == headPos:
                    while applePos in snake or applePos == headPos:
                        applePos = random.randrange(0, len(map)-1)

                # clears canvas
                C.delete("all")

                # displayes grid
                for i in range(0, len(map)):
                    x, y = mapToCoord(i, gridWidth)
                    x *= squareWidth
                    y *= squareWidth
                    C.create_rectangle(x, y, x + squareWidth, y +
                                       squareWidth, fill="lightGreen")

                # displayes the snakes body
                for body in snake:
                    map[i] = 1
                    x, y = mapToCoord(body, gridWidth)
                    x *= squareWidth
                    y *= squareWidth

                    C.create_rectangle(x, y, x + squareWidth, y +
                                       squareWidth, fill="darkGreen")

                # displayes the snakes head
                x, y = mapToCoord(headPos, gridWidth)
                x *= squareWidth
                y *= squareWidth
                C.create_rectangle(x, y, x + squareWidth, y +
                                   squareWidth, fill="purple")

                # displayes the apple
                x, y = mapToCoord(applePos, width)
                x *= squareWidth
                y *= squareWidth
                C.create_rectangle(x, y, x + squareWidth, y +
                                   squareWidth, fill="red")

                # updates the display and waits

                C.create_text(widowWidth/2, widowWidth/10,
                              justify='center', text=score, font=20)
                C.update()
                time.sleep(0.2)
            mainloop()

    elif vizAI:
        while True:

            prevHeadPos = headPos

            # moves the head acording to direction

            headPos, path = AstarMove(
                map, gridWidth, headPos, snake, path, applePos)

            # if snakeMove returns "illegal move" the game ends
            if headPos == "illegal move":
                print("you Lose because headpos")
                break

            # if the head is not on the apple the last grid on snake dissaperes, otherwhise it remains
            if headPos != applePos:
                if len(snake) > 0:
                    snake.pop(0)
                snake.append(prevHeadPos)
            else:
                snake.append(prevHeadPos)
                score += 1

            # when the head is eaten a new one will appere in a random position,
            # if its appers on the snake a new position is generated
            if applePos == headPos:
                while applePos in snake or applePos == headPos:
                    applePos = random.randrange(0, len(map))

            # if the snake intersects with itself the game ends
            if headPos in snake:  # or headPos == prevHeadPos
                print("you Lose because head in snake")
                print("Snake", snake)
                break

            # clears canvas
            C.delete("all")

            # displayes grid
            for i in range(0, len(map)):
                x, y = mapToCoord(i, gridWidth)
                x *= squareWidth
                y *= squareWidth
                C.create_rectangle(x, y, x + squareWidth, y +
                                   squareWidth, fill="lightGreen")

            # displayes the snakes body
            for body in snake:
                map[i] = 1
                x, y = mapToCoord(body, gridWidth)
                x *= squareWidth
                y *= squareWidth

                C.create_rectangle(x, y, x + squareWidth, y +
                                   squareWidth, fill="darkGreen")

            # displayes the snakes head
            x, y = mapToCoord(headPos, gridWidth)
            x *= squareWidth
            y *= squareWidth
            C.create_rectangle(x, y, x + squareWidth, y +
                               squareWidth, fill="purple")

            # displayes the apple
            x, y = mapToCoord(applePos, width)
            x *= squareWidth
            y *= squareWidth
            C.create_rectangle(x, y, x + squareWidth, y +
                               squareWidth, fill="red")

            # updates the display and waits

            C.create_text(widowWidth/2, widowWidth/10,
                          justify='center', text=score, font=20)
            C.update()
        mainloop()
    return score


width = 12
map = generateMap(width)

print(visualize(1000, map, width, False, vizAI=True))
