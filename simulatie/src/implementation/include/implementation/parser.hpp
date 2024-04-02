#ifndef PARSER_HPP
#define PARSER_HPP


#include <string>
#include <vector>


// The unsigned shorts are index and pwm respectively
using pair = std::pair<unsigned short, unsigned short>; 

// The first part is the joint movements (with index and pwm) and the second part is the time
using fullcommand = std::pair<std::vector<pair>, unsigned short>; 


class Parser
{
public:

    Parser();
    ~Parser();

   bool parseCommand(std::string& command);

   const fullcommand& getMovementsAndTime() const;

private:

    bool parseMovementCommand(std::string& command, std::vector<pair>& parsedCommands);
    
    bool checkValidity(std::vector<pair>& parsedCommands, signed short time);

    fullcommand movementsAndTime;

};




#endif