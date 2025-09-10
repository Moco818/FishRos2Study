#include <iostream>
#include <memory>
using namespace std;

int main(int argc, char const *argv[])
{
    auto p1 = make_shared<string>("This is a str.");
    cout << "p1's quoto counnt is: " << p1.use_count() << ", it's memory send to memory is: " 
        << p1.get() << endl;

    auto p2 = p1;
    cout << "p1's quoto counnt is: " << p1.use_count() << ", it's memory send to memory is: " 
        << p1.get() << endl;
    cout << "p2's quoto counnt is: " << p2.use_count() << ", it's memory send to memory is: " 
        << p2.get() << endl;

    p1.reset();

    cout << "p1's quoto counnt is: " << p1.use_count() << ", it's memory send to memory is: " 
        << p1.get() << endl;
    cout << "p2's quoto counnt is: " << p2.use_count() << ", it's memory send to memory is: " 
        << p2.get() << endl;

    cout << "p2 send to resources is: " << p2 -> c_str() << endl;
    return 0;
}
