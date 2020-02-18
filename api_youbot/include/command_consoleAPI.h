/*
 * command_console.h
 *
 *  Created on: Dec 4, 2016
 *      Author: Kourosh
 */


#ifndef COMMAND_CONSOLE_H_
#define COMMAND_CONSOLE_H_

#include <control_youbot/ctrl_controller_interface.h>
#include <control_youbot/futils.h>
#include <control_youbot/ortosdata_defines.h>

using std::cout;
using std::endl;

std::map<std::string, int> armStr2Int = { { "left", 0 }, { "right", 1 } };

class CommandConsole
{
	ortos::xcom::XCOMInterface* xcom_;
	CTRL::ControllerInterface ctrlInterface_;
public:
	CommandConsole();
	virtual ~CommandConsole();

	enum class ParsRet
	{
		Ok, CmdNotRecognized, InvalidArm, InvalidInputVals, InvalidCtrlMode
	};

	const std::map<ParsRet, std::string> ParsRetNames = { { ParsRet::Ok, "Ok" }, { ParsRet::InvalidArm, "Invalid Arm" }, {
			ParsRet::CmdNotRecognized, "Command Not Recognized" }, { ParsRet::InvalidInputVals, "Invalid Input Values" }, {
			ParsRet::InvalidCtrlMode, "Invalid Ctrl Mode" } };

	void PrintCommandList();
	ParsRet ParseInput(const std::string input, ortosdata::CommandType &cmd, int &arm, std::vector<double> &values,
			CTRL::CtrlMode &ctrlMode);
	bool ValuesAreValid(const ortosdata::CommandType &cmd, const std::vector<double> &values);
	void SendCommand(const ortosdata::CommandType cmd, const int arm,
			const std::vector<double> &cmdValues, const CTRL::CtrlMode ctrlMode);
};

#endif /* CONTROL_BAXTER_INCLUDE_COMMAND_CONSOLE_H_ */

