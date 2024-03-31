classdef houseObj < handle

    properties
        ID % name of unit
        cmds % dictionary (cmdName, cmd obj), stores cmds from Arduino getCommands()
        cmdMenu % dictionary (cmdName, cmd description), preprogrammed in private properties
        outPromptMenu % dictionary (outCmds, outPrompts), preprogrammed in private properties
        inPromptMenu % dictionary (inCmds, inPrompts), preprogrammed in private properties
        rangeMenu % dictionary (promptCmds, cmdRange), preprogrammed in private properties
        
        SP % Serial port obj for communication
        status % values returned from Arduino getAll()
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
            obj.status = strings; % string array
        end % constructor

        % setup a house object
        function setup(obj, serialPort, ID)
            obj.ID = ID;
            obj.SP = serialPort;
            obj.setUpCmds(); % populate obj.cmds
        end % setup()
        
        % set up the class property, obj.cmds
        function setUpCmds(obj)
            % load cmds from microcontroller
            obj.writeSP("getCommands");
            cmdName = obj.readSP(); % read a cmd name
            % keep getting cmds until "eoc"
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
                    end % if for setting input and output prompts
                end

                

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
        end

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
        function cmdMessage = call(obj, cmdName)
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
                cmdMessage = obj.readCmdMessage(currCmd);
            elseif (currCmd.out ~= 0) % cmd needs user input
                obj.sendCmdArguments(currCmd);
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
            input = str2double(input);
            if isnan(input)
                flag = 0;
                disp("* Warning: Input is not a number, try again.");
            end
            
            if (~isreal(input) || input ~= floor(input)) % check int
                flag = 0;
                disp("* Warning: Input is not an integer, try again.");
            end
            
            % invalid input if out of range
            if(input < currCmd.minRange || input > currCmd.maxRange)
                flag = 0;
                disp("* Warning: Input out of range, try again.");
            end
            isValid = flag;
        end % isvalid()

        % send out cmd with its arguments
        function sendCmdArguments(obj, cmdName, args)
            % commented out due to customized cmds
            %if isKey(obj.outPromptMenu, cmdName) == 0
            %    str = "ERROR: " + "'" + cmdName + "' is not in outPromptMenu";
            %    error(str);
            %end

            currCmd = obj.cmds(cmdName);
            str = cmdName + newline;
            % append arguments
            for i = 1:currCmd.out
                str = str + string(args(i)); 
                if i ~= currCmd.out
                    str = str + newline;
                end
            end
            % send cmd
           obj.writeSP(str)
        end % end sendCmdArguments()

        % update excel with the lastest Arduino getAll() data
        function updateStatus(obj)
            if obj.SP.BytesAvailable ~= 0 % if serial port is busy
                writematrix(obj.status, "statusData.xlsx", "Sheet", "Grid", "writeMode", "append");
                writematrix(obj.status, "statusData.xlsx", "Sheet", obj.ID, "writeMode", "append");
                return;
            end

            count = obj.cmds("getAll").in; % number of input
            obj.writeSP("getAll"); % send getAll cmd
            str = strings; % str is a string array
            % format a string array to put in excel
            % append time
            str(1) = string(datetime);
            str(2) = obj.ID; 
            % append getAll values
            for i = 1:count
                str(i + 2) = obj.readSP();
            end
            % append temperature
            obj.writeSP("getTemp"); % send getPos cmd
            str(count + 3) = obj.readSP();
            obj.status = str; % update obj status
            %disp(obj.ID);
            %disp(obj.status);
            %disp(newline);
            % write to excel
            writematrix(obj.status, "statusData.xlsx", "Sheet", "Grid", "writeMode", "append");
            writematrix(obj.status, "statusData.xlsx", "Sheet", obj.ID, "writeMode", "append");
        end % end updateStatus()


        % read cmd message sent back from MATLAB
        function cmdMessage = readCmdMessage(obj, currCmd)
            % commented out for customized cmds
            %if isKey(obj.inPromptMenu, currCmd.name) == 0
            %    str = "ERROR: " + "'" + currCmd.name + "' is not in inPromptMenu";
            %    error(str);
            %end
            cmdMessage = strings;
            % send cmd
            obj.writeSP(currCmd.name);
            % read all inputs
            for i = 1:(currCmd.in)
                str = obj.readSP();
                if currCmd.prompts == "unknown: "
                    prompt = "";
                else
                    prompt = currCmd.prompts(i) + newline + "      ";
                end
                str = strcat(prompt, str, newline);
           
                cmdMessage(i) = str;
                % disp(str);
            end
            % disp(newline);
        end % readCmdMessage()
    end % method
end % houseObj class















