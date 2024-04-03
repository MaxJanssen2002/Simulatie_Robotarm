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

   /// @brief Parses an incoming command based on the SSC-32U format
   /// @param command The incoming command
   /// @return True if the command is valid, false if not
   bool parseCommand(std::string& command);

   const fullcommand& getMovementsAndTime() const;

private:

    /// @brief Parses a part of the command that should give one index and pwm (e.g. #2 P1200)
    /// @param command The part of the command
    /// @param parsedCommands The vector of commands which gets a new element if the command is valid
    /// @return True if the part of the command is valid, false if not
    bool parseMovementCommand(std::string& command, std::vector<pair>& parsedCommands);
    
    /// @brief Performs a final check for the new vector of parsed commands
    /// @param parsedCommands The vector of commands
    /// @param time The duration that has been parsed from the command
    /// @return True if the command is valid, false if not
    bool checkValidity(std::vector<pair>& parsedCommands, signed short time);

    fullcommand movementsAndTime;

};




#endif