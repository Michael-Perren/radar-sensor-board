import gdb
import csv
import string
def main():
    print("Functions:\n--getdata(nbatches, filename): writes nbatches of fftmagnitude outputs to a csv called filename")
     # Write header 


def getfftmagarray(data):
    Frame = gdb.selected_frame()
    var = Frame.read_var("mag")
    for i in range(512):
        data.append(int(var[i]))
    return data

def outputcsv(numlists, listoflists, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            for i in range(512):
                row = []
                for j in range(len(listoflists)):
                    row.append(listoflists[j][i])
                writer.writerow(row)
   

def getdata(nbatches, filename):
    gdb.execute("b app_freertos.c:269")
    severallists = list()
    for i in range(int(nbatches)):
        gdb.execute("continue")
        maglist = []
        getfftmagarray(maglist)
        severallists.append(maglist)
    print(len(severallists))
    outputcsv(nbatches,severallists, filename)

if __name__ == "__main__":
    main()