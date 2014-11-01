#include <iostream>
#include <unistd.h>
#include <vector>



using namespace std;

int main () {
   
    std::vector<double>  coords (4);

    cin >> coords.at(0);
    cin >> coords.at(1);
    cin >> coords.at(2);
    cin >> coords.at(3);

    cerr << "rx child : " << coords.at(0) << coords.at(1) << coords.at(2) << coords.at(3)<< endl;
    cout << "99" << endl;
}

