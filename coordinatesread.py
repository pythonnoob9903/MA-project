from pathlib import Path
def getcords():
    p = Path(__file__).with_name('coordinates.txt')

    Xcord = []
    Ycord = []
    Zcord = []

    coordinates = p.open('r')
    lines = coordinates.readlines()
    coordinates.close()

    for i in range(len(lines)):
        counter1 = 0
        while lines[i][counter1] != ";":
            counter1 += 1
        Xcord += [lines[i][0:counter1]]
        counter2 = counter1 + 1
        while lines[i][counter2] != ";":
            counter2 += 1
        Ycord += [lines[i][counter1+1:counter2]]
        Zcord += [lines[i][counter2+1:-1]]
    return Xcord, Ycord, Zcord