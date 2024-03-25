#include "implementation/parser.hpp"

#include <algorithm>
#include <sstream>
#include <iostream>


Parser::Parser() {}


Parser::~Parser() {}


bool Parser::parseCommand(std::string& command)
{
    // Remove all whitespaces
    command.erase(remove_if(command.begin(), command.end(), isspace), command.end());

    // Error if there is less or more than one 'T' in the command
    if(std::count_if(command.begin(), command.end(), [](unsigned char c) { return c == 'T'; }) != 1)
    {
        std::cout << "The command should contain one 'T'." << std::endl;
        return;
    }

    std::vector<pair> movements;
    signed short time = -1;
    std::string result;

    std::istringstream fullCommand(command);
    while (std::getline(fullCommand, result, 'T')) // Separate the time from the rest of the command
    {
        if (!result.empty() && std::find_if(result.begin(), result.end(), [](unsigned char c) { return !std::isdigit(c); }) == result.end())
        {
            time = stoi(result);
        }
        else
        {
            std::istringstream commandWithoutTime(result);
            while (std::getline(commandWithoutTime, result, '#')) // Separate the hashtags to parse the index and pwm for each movement
            {
                if (!parseMovementCommand(result, movements)) // If the function returns false, there's an error, so stop parsing
                {
                    return;
                }
            }
        }
    }

    if (!checkValidity(movements, time))
    {
        return;
    }

    movementsAndTime = std::make_pair(movements, time);
}


const fullcommand& Parser::getMovementsAndTime() const
{
    return movementsAndTime;
}
    


bool Parser::parseMovementCommand(std::string& command, std::vector<pair>& parsedCommands)
{
    std::string result;
    signed short index = -1;
    signed short pwm = -1;

    // An empty command doesn't mess with the syntax, so do nothing
    if (command.empty())
    {
        return true;
    }

    // Error if there is less or more than one 'P' after the hashtag
    if (std::count_if(command.begin(), command.end(), [](unsigned char c) { return c == 'P'; }) != 1)
    {
        std::cout << "You should have one 'P' after each hashtag." << std::endl;
        return false;
    }

    std::istringstream indexPWMseparator(command);
    while (std::getline(indexPWMseparator, result, 'P')) //Separate the index and the pwm of the command
    {
        if (!result.empty() && std::find_if(result.begin(), result.end(), [](unsigned char c) { return !std::isdigit(c); }) == result.end())
        {
            if (index == -1)
            {
                index = stoi(result);
            }
            else
            {
                pwm = stoi(result);
            }
        }
        else
        {
            std::cout << "'" << result << "' is not a number." << std::endl;
            return false;
        }
    }

    // Error if the time or the pwm has never been changed due to wrong syntax
    if (index == -1 || pwm == -1)
    {
        std::cout << "You seem to be missing a number after a '#' or a 'P'." << std::endl;
        return false;
    }

    parsedCommands.emplace_back(std::make_pair(index, pwm));

    return true;
}


bool Parser::checkValidity(std::vector<pair>& parsedCommands, signed short time)
{
     // Error if the time has never been changed due to wrong syntax
    if (time == -1)
    {
        std::cout << "The part after the 'T' is either empty, or it isn't numeric." << std::endl;
        return false;
    }

    // Error if the vector of joint movements is still empty
    if (parsedCommands.empty())
    {
        std::cout << "You know that you didn't give any PWM commands to the joints, right?" << std::endl;
        return false;
    }

    // Error if there are any duplicate indexes
    std::sort(parsedCommands.begin(), parsedCommands.end(), [](pair a, pair b) { return a.first < b.first; });
    if (std::adjacent_find(parsedCommands.begin(), parsedCommands.end(), [](pair a, pair b) { return a.first == b.first; }) != parsedCommands.end())
    {
        std::cout << "Duplicate index" << std::endl;
        return false;
    }

    return true;
}