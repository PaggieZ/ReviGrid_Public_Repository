classdef houseObj < handle

    properties
        ID % name of unit
        cmds % dictionary (cmdName, cmd obj), stores cmds from Arduino getCommands()
        cmdMenu % dictionary (cmdName, cmd description)
        outPromptMenu % dictionary (outCmds, outPrompts)
        inPromptMenu % dictionary (inCmds, inPrompts)
        rangeMenu % dictionary (promptCmds, cmdRange)
        modeFlag
        
        SP % Serial port obj for communication
    end

    properties (Access = private)
        % name of all house cmds
        cmdList = ["*ID?" "getCommands" "init" ...
            "lightAll" "lightsOut" ...
            "autoOn" "autoOff" ...
            "light0" "light1" "light2" "light3" ...
            "blinkHouses" ...
            "chaseOn" "chaseOff" ...
            "setLimits" "setLoad"... 
            "setLight0", "setLight1", "setLight2", "setLight3", ...
            "getAll" "getLoads" "getLoadVal" "getKW" "getCarbon" ...
            "getTemp","getHumidity","getPressure" ...
            "off" ...
            ]
        % description of all house cmds
        cmdDef = ["[private] returns unit ID" ... % *ID?
            "[private] returns all available cmds on Arduino" ... % getCommands
            "sets the unit to its initial condition" ... % init
            "turns on all lights in the four houses, overwrites all house load limits to 1" ... % lightAll
            "turns off lights in all houses, overwrites all house load limits to 0" ... % lightsOut
            "initiates random house lighting based on house load limits" ... % autoOn
            "turns off random house lighting" ... % autoOff
            "lights up house 0" ... % light0
            "lights up house 1" ... % light1
            "lights up house 2" ... % light2
            "lights up house 3" ... % light3
            "blinks the houses three times" ... % blinkHouses
            "turns on the chase pattern of lights" ... % chaseOn
            "turns off the chase pattern" ... % chaseOff
            "determines if a house load can be turned on or not, modifies each house load limit" ... % setLimits
            "sets the load of each houses" ... % setLoad
            "sets the light level of house 0" ... % setLight0
            "sets the light level of house 1" ... % setLight1
            "sets the light level of house 2" ... % setLight2
            "sets the light level of house 3" ... % setLight3
            "returns 7 numbers from the house unit" ... % getAll
            "returns current load settings" ... % getloads
            "returns the load value of the combined load of all lit houses" ... % getLoadVal
            "returns the kilowatt value of all the lit houses" ... % getKW
            "returns carbon emissions from the houses in tons" ... % getCarbon
            "returns temperature" ... % getTemp
            "returns humidity" ... % getHumidity
            "returns pressure" ... % getPressure
            "turns the unit off" ... % off
            ]

        % cmds that send arguments to unit
        outCmds = ["setLimits" "setLoad" "setLight0" "setLight1" "setLight2" "setLight3"];
        % outCmd prompts
        outPrompts = {["h0Lim (0 or 1): " "h1Lim (0 or 1): " "h2Lim (0 or 1): " "h3Lim (0 or 1): "] ... % setLimits
                      ["house load (0 - 500): "] ... % setLoad
                      ["house0 light level (0 - 4): "] ... % setLight0
                      ["house1 light level (0 - 4): "] ... % setLight1
                      ["house2 light level (0 - 4): "] ... % setLight2
                      ["house3 light level (0 - 4): "] ... % setLight3
                     };
        % user input range
        cmdRange = {[0 1] ... % setLimits
                    [0 500] ... % setLoad
                    [0 4] ... % setLight0
                    [0 4] ... % setLight1
                    [0 4] ... % setLight2
                    [0 4] ... % setLight3
                   };

        % cmds that receive information from unit
        inCmds = ["getAll" "getLoads" "getLoadVal" "getKW" "getCarbon" "getTemp" "getHumidity" "getPressure"];
        % inCmd prompts
        inPrompts = {["Kilowatt capacity: " "Current KW level: " "Load allocated: " "Delta Load: " ...
                      "Carbon value: " "Renewability: " "Current Power: "] ... % getAll
                      ["h1 load: " "h2 load: " "h3 load: " "h4 load: "] ... % getLoads
                      ["Combined load: "] ... % getLoadVal
                      ["KW: "] ... % getKW
                      ["Carbon emission in ton: "] ... % getCarbon
                      ["Temperature: "] ... % getTemp
                      ["Humidity: "] ... % getHumidity
                      ["Pressure: "] ... % getPressure
                    }
    end % private property

    methods
        % constructor
        function obj = houseObj()
            % create command menu (cmdName, cmdDefinition)
            if size(obj.cmdList, 2) ~= size(obj.cmdDef, 2) 
                error("ERROR: Invalid command menu");
            end
            obj.cmdMenu = dictionary(obj.cmdList, obj.cmdDef); 

            % create outPrompt menu (promptCmd, outPrompts)
            if (size(obj.outCmds, 2) ~= size(obj.outPrompts, 2))
                error("ERROR: Invalid outPrompt menu");
            end
            obj.outPromptMenu = dictionary(obj.outCmds, obj.outPrompts);

            % create range menu (outCmd, userInputRange)
            if (size(obj.outCmds, 2) ~= size(obj.cmdRange, 2))
                error("ERROR: Invalid range menu");
            end
            obj.rangeMenu = dictionary(obj.outCmds, obj.cmdRange); 

            % create inPrompt menu (inCmd, inPrompts)
            if (size(obj.inCmds, 2) ~= size(obj.inPrompts, 2))
                error("ERROR: Invalid inPrompt menu");
            end
            obj.inPromptMenu = dictionary(obj.inCmds, obj.inPrompts); 

            obj.cmds = dictionary; % stores cmds available for users to call
        end % constructor

        % setup the object
        function setup(obj, serialPort, ID, modeFlag)
            obj.ID = ID;
            obj.SP = serialPort;
            obj.setUpCmds(); % populate obj.cmds
            obj.modeFlag = modeFlag;
        end % setup()
        

        % set up the class property, obj.cmds
        function setUpCmds(obj)
            % load cmds from microcontroller
            obj.writeSP("getCommands");
            cmdName = obj.readSP(); % read a cmd name
            % keep reading cmds until "eoc"
            while strcmp('eoc', cmdName) == 0
                numOfOutput = 0;
                numOfInput = 0;
                specialIndex = -1; % index of the 1st special char
                                   % ">" means cmd needs input
                                   % "<" means cmd has output
                % if name contains ">"
                if contains(cmdName, ">") == 1
                    % update num of output
                    specialIndex = strfind(cmdName, ">");
                    numOfOutput = cmdName(specialIndex + 1:end);
                    numOfOutput = str2double(numOfOutput);
                end
                % if name contains "<"
                if contains(cmdName, "<") == 1
                    % update name of input
                    specialIndex = strfind(cmdName, "<");
                    numOfInput = cmdName(specialIndex + 1:end);
                    numOfInput = str2double(numOfInput);
                end
                % if name has special char
                if specialIndex ~= -1
                    % update name
                    cmdName = cmdName(1:specialIndex - 1);
                end
                

                % create cmd obj
                currCmd = cmd(cmdName, numOfInput, numOfOutput);
                currCmd.setUnitName(obj.ID);

                % set cmd parameters
                if isKey(obj.cmdMenu, cmdName) == 0 % cmd is not in obj.cmdMenu
                    % generate a warning when cmd is not in dictionary
                    str = "Warning: " + cmdName + " is not in command menu.";
                    disp(str);

                    currCmd.setDescription("unknown"); 
                    currCmd.setPrompt(["unknown: "]);
                    currCmd.setRange(NaN, NaN);
                else % cmd is in obj.cmdMenu
                    currCmd.setDescription(obj.cmdMenu(cmdName));
                    % set prompts for cmds with input and output
                    if numOfOutput ~= 0 % cmd has output
                        % error checking, cmd is not in dictionary
                        if isKey(obj.outPromptMenu, cmdName) == 0
                            str = "ERROR: " + cmdName + " is not in outPrompt menu.";
                            error(str);
                            return;
                        end
    
                        % set prompts
                        cmdPrompt = obj.outPromptMenu(cmdName);
                        cmdPrompt = cmdPrompt{1};
                        if numOfOutput ~= size(cmdPrompt, 2)
                            str = "ERROR: Invalid number of output for '" + cmdName + "'";
                            error(str);
                        end
                        currCmd.setPrompt(cmdPrompt);
                        % set range
                        cmdRange = obj.rangeMenu(cmdName);
                        min = cmdRange{1}(1);
                        max = cmdRange{1}(2);
                        currCmd.setRange(min, max);
                    elseif numOfInput ~= 0 % cmd has input
                        % error checking, cmd is not in dictionary
                        if isKey(obj.inPromptMenu, cmdName) == 0
                            str = "ERROR: '" + cmdName + "' is not in inPrompt menu.";
                            error(str);
                        end
    
                        % set prompts
                        cmdPrompt = obj.inPromptMenu(cmdName);
                        cmdPrompt = cmdPrompt{1};
                        if numOfInput ~= size(cmdPrompt, 2)
                            str = "ERROR: Invalid number of input for '" + cmdName + "'";
                            error(str);
                        end
                        currCmd.setPrompt(cmdPrompt);
                    end % if, setting input and output prompts
                end % if, cmd in or not in cmdMenu

                % store cmd obj in dictionary
                obj.cmds(cmdName) = currCmd;
                % get next cmd name
                cmdName = obj.readSP();
            end % while loop
        end % setUpCmds()


        % write a string to serial port
        function writeSP(obj, str)
            str = string(str) + newline;
            obj.SP.write(str, "char");
        end % writeSP()


        % read and format a serial input
        function str = readSP(obj)
            % check serial port readline
            try
                lastwarn('');
                str = readline(obj.SP); % read a message from serial buffer
                if(~isempty(lastwarn))
                    % readline() only triggers a warning when timeout
                    % insert an error to trigger "catch"
                    error();
                end
            catch 
                str = "ERROR: Can't read ID from " + obj.SP.Port;
                error(str);
            end

            str = convertStringsToChars(str); % convert to char vector for formatting
            carriageReturn = '';
            carriageReturn(1) = 13;
            while(endsWith(str, newline) || endsWith(str, carriageReturn))
                str = str(1:end - 1); % remove terminator
            end
        end % end readSP()


        % send a cmd to Arduino
        function returnedVal = call(obj, cmdName, args)
            % error checking, cmd is not in dictionary
            if isKey(obj.cmds, cmdName) == 0
                str = "ERROR: " + "'" + cmdName + "' is not in cmd menu.";
                error(str);
                return;
            end
            % find cmd obj from dictionary
            currCmd = obj.cmds(cmdName);
            % send cmds
            if (currCmd.in == 0 && currCmd.out == 0) % cmds without input and output
                obj.writeSP(cmdName);
            elseif (currCmd.in ~= 0) % Matlab receives input
                returnedVal = obj.readCmdMessage(currCmd);
            elseif (currCmd.out ~= 0) % cmd needs user input
                if obj.modeFlag == 0
                    obj.sendCmdArgumentDeveloper(currCmd, args);
                else
                    obj.sendCmdArgumentUser(currCmd)
                end
            end % if
        end % call()

        % check if a user input is valid or not
        function isValid = checkValid(obj, currCmd, input)
            % skip the validity check if range is unknown (for customized
            % cmds)
            if isnan(currCmd.maxRange)
                isValid = 1;
                return
            end
            
            flag = 1;
            if obj.modeFlag == 1
                input = str2double(input);
            end
            if isnan(input)
                flag = 0;
                str = "* Warning: Input is not a number.";
                if obj.modeFlag == 0 % developer mode
                    error(str);
                else % User mode
                    disp(str);
                end
            end
            % invalid input if is not an integer
            if(~isreal(input) || input ~= floor(input))
                flag = 0;
                str = "* Warning: Input is not an integer.";
                if obj.modeFlag == 0 % developer mode
                    error(str);
                else % User mode
                    disp(str);
                end
            end
            
            % invalid input if out of range
            if(input < currCmd.minRange || input > currCmd.maxRange)
                flag = 0;
                str = "* Warning: Input out of range. [" + string(currCmd.minRange) + ' ' + string(currCmd.maxRange) + ']' + newline;
                if obj.modeFlag == 0 % developer mode
                    error(str);
                else % User mode
                    disp(str);
                end
            end
            isValid = flag;
        end % checkValid()

        % !!! different from App designer objects
        % get cmd arguments from user and send them to the unit
        function sendCmdArgumentUser(obj, currCmd)
            strOut = convertCharsToStrings(currCmd.name) + newline;
            % receive all user input
            for i = 1:(currCmd.out)
                if isKey(obj.cmdMenu, currCmd.name) == 0
                    prompt = "input: ";
                else
                    prompt = currCmd.prompts{i};
                end
                isValid = 0;
                while isValid == 0
                    str = input(prompt, "s");
                    isValid = obj.checkValid(currCmd, str);
                end
                str = convertCharsToStrings(str) + newline;
                strOut = strOut + str;
            end % for, take user inputs
            obj.writeSP(strOut);
        end % sendCmdArgumentUser()

        % get cmd arguments from user and send them to the unit
        function sendCmdArgumentDeveloper(obj, currCmd, args)
            
            if numel(args) ~= currCmd.out
                str = "ERROR: Invalid arguments.";
                error(str);
            end

            strOut = convertCharsToStrings(currCmd.name) + newline;
            % check input validity
            isValid = 0;
            for i = 1:(currCmd.out)
                isValid = obj.checkValid(currCmd, args(i));
                str = convertCharsToStrings(args(i));
                strOut = strOut + str + newline;
            end
            obj.writeSP(strOut);
        end % sendCmdArgumentDeveloper()

        % read cmd message send back to MATLAB
        function returnedVal = readCmdMessage(obj, currCmd)
            % send cmd
            obj.writeSP(currCmd.name);
            returnedVal = strings;
            % receive all input
            if obj.modeFlag == 0 % developer mode
                % send cmd
                obj.writeSP(currCmd.name);
                % receive all input
                for i = 1:(currCmd.in)
                    str = obj.readSP(); % read a serial input
                    returnedVal(i) = str;
                end
            else % User mode
                for i = 1:(currCmd.in)
                    str = obj.readSP(); % read a serial input
                    returnedVal(i) = str;
                    if (currCmd.prompts == "unknown: ")
                        prompt = "";
                    else
                        prompt = currCmd.prompts(i);
                    end
                    str = strcat(prompt, newline + "   ", str);
                    disp(str);
                end
            end
        end % readCmdMessage()
    end % methods
end % houseObj class















