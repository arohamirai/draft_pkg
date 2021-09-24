/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-09-24 19:42:19
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-09-24 19:47:32
 * @Description: 
 */
#include "nlohmann/json.hpp"
#include <vector>
#include <iostream>
using namespace std;
int main()
{
    std::vector<float> vec = {1.1, 2.2, 3.3, 4.4, 5.5, 6.0};
    nlohmann::json js(vec);
    std::cout << js.dump();
}