#include <iostream>
using namespace std;

int main(){
    const char * a = "Hello word!";
    char * const b = "wang"; 
    cout << "hello world!" << endl;
    cout << a << endl;
    cout << b << endl;
    cout << *a << endl;
    return 0;
}