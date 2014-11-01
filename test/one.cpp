#include <iostream>
#include <unistd.h>
 #include <stdlib.h>
#include <fstream>
#include <vector>

using namespace std;

int main() {
    int x;
    int toParent[2], fromParent[2];
    pipe(toParent);
    pipe(fromParent);

    if (fork()) { // parent
        close(toParent[1]);
        close(fromParent[0]);
    }
    else {
        close(toParent[0]);
        close(fromParent[1]);
        dup2(toParent[1], 1);
        dup2(fromParent[0], 0);
        close(toParent[1]);
        close(fromParent[0]);
        execl("two", "two", (char *) NULL);
    }

    FILE * output = fdopen(fromParent[1], "w");
    FILE * input = fdopen(toParent[0], "r");
    fprintf(output, "%d\n%d\n%d\n%d\n", 32,54,100,1);
    fflush(output);
    fscanf(input,"%d", &x);
    cout << "rx " << x << endl; 
}