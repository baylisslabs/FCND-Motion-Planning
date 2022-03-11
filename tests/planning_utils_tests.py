from time import sleep
import unittest

from planning_utils import generate_line

def newGrid(rows,cols):
    if rows:
        return [[0]*cols]+newGrid(rows-1,cols)
    return []

def printGrid(grid):
    for r in grid:
        print(r)

class TestGenerateLine(unittest.TestCase):
    def generate_perimiter(self,rows,cols):
        for x in range(0,cols):
            yield (x,0)
        for y in range(1,rows):
            yield (cols-1,y)
        for x in range(2,cols+1):
            yield (cols-x,rows-1)
        for y in range(2,rows+1):
            yield (0,rows-y)

    def testSweep(self):
        rows, cols = 25, 25
        x0,y0 = (cols//3, rows//3)
        x1 = cols-1
        for x1,y1 in self.generate_perimiter(rows,cols):
            grid = newGrid(rows, cols)
            for x,y in generate_line((x0,y0),(x1,y1)):
                grid[y][x]=1
            print(chr(27) + "[2J")
            printGrid(grid)
            sleep(0.1)

    def testPoint(self):
        rows, cols = 25, 25
        x0,y0 = (cols//2, rows//2)
        x1,y1 = (cols//2, rows//2)
        grid = newGrid(rows, cols)
        for x,y in generate_line((x0,y0),(x1,y1)):
            grid[y][x]=1
        printGrid(grid)

if __name__ == '__main__':
    unittest.main()

