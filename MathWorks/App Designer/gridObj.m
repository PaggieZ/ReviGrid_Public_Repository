classdef gridObj < handle
    %GRIDOBJ Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        SPs % all available serial ports
        unitCount % num of available serial ports (num of available units)
        unitList % stores all available units
        unitIDs % sotres all available unit ID
        FID % "log.txt" file ID for display
        baudRate % available baudRates
        
        cmdMenu % dictionary (cmd name, cmd description)
    end

    properties (Access = private)
        % name of all grid cmds
        cmdList = ["init" "off"]
        % description of all grid cmds
        cmdDef = ["initializes all units" ... % init
                  "turns off all units" ... % off
                 ]
    end
    
    methods
        % constructor
        function obj = gridObj()
            obj.SPs = serialportlist; % get all available serial ports
            obj.unitCount = size(obj.SPs, 2); % get num of available units
            obj.unitList = {};
            obj.unitIDs = strings;
            obj.setupAll(); % set up all of the unit objs in grid
            obj.unitCount = size(obj.unitList, 2); % get num of available units
            obj.cmdMenu = dictionary(obj.cmdList, obj.cmdDef);
        end % constructor

        function setFID(obj, FID)
            obj.FID = FID;
        end

        % reports all unit names and their serial port name
        function reportGridStatus(obj)
            str = "* Grid Created" + newline;
            for i = 1:obj.unitCount
                str = str + "   -" + obj.SPs(i) + ": " + obj.unitIDs(i) + newline;
            end
            fprintf(obj.FID, str);
        end % reportFridStatus()

        % write a string to serial port
        function writeSP(obj, SP, str)
            str = string(str) + newline;
            SP.write(str, "char");
        end

        % read and format a serial input
        function str = readSP(obj, SP)
            str = readline(SP); % read a message from serial buffer
            str = convertStringsToChars(str); % convert to char vector for formatting
            carriageReturn = '';
            carriageReturn(1) = 13;
            while(endsWith(str, newline) || endsWith(str, carriageReturn))
                str = str(1:end - 1); % remove terminator
            end
        end % end readSP()

        % read ID from a module
        function ID = readID(obj,SP)
             % check serial port readline
            try
                lastwarn('');
                ID = readline(SP); % get ID
                if(~isempty(lastwarn))
                    % readline() only triggers a warning when timeout
                    % insert an error to trigger "catch"
                    error();
                end
            catch 
                str = "ERROR: Can't read ID from " + SP.Port;
                warning(str);
                return;
            end

            % check ID validity
            if isstring(ID) == 0
                str = "ERROR: Invalid ID from " + SP.Port; 
                warning(str);
                return;
            end
            % remove newline char from ID
            ID = convertStringsToChars(ID);
            ID = ID(1:end - 1);
        end

        % set up all avaialble objects
        function setupAll(obj)
            for i = 1:obj.unitCount % for each serial port
                str = "** Registering serial port " + obj.SPs(i);
                disp(str);
                % register the port
                try
                    s = serialport(obj.SPs(i), 115200, "Timeout", 2); % 2 sec timeout
                    pause(1.5); % wait for serial port set up, min 1.5
                catch
                    str = "ERROR: Can't read ID from " + obj.SPs(i);
                    warning(str);
                    continue;
                end
                configureTerminator(s, "LF"); % newline terminator for both read and write
                obj.writeSP(s, "*ID?");
                ID = obj.readID(s);
                if(~isempty(lastwarn)) % skip the serial port if ID is invalid
                    continue;
                end
                if ID == "houseload" % set up house object
                    unit = houseObj;
                elseif ID == "solartracker" % set up solar tracker object
                    unit = solarObj;
                elseif ID == "windturbine" % set up wind turbine object
                    unit = windObj;
                elseif ID == "fan" % set up fan object
                    unit = fanObj;
                elseif ID == "generator" % set up generator object
                    unit = generatorObj;
                end % if
                unit.setup(s, ID);
                unitIndex = length(obj.unitList) + 1;
                obj.unitList{unitIndex} = unit;
                obj.unitIDs(unitIndex) = ID;
                str = ID + " registered" + newline;
                disp(str);
                obj.writeSP(s, "init");
            end % for
        end % setupAll()

        % update the status of all units in the grid
        function updateStatusAll(obj)
            for i = 1:obj.unitCount
                currUnit = obj.unitList{i};
                % disp(currUnit.ID);
                % disp(currUnit.SP.BytesAvailable);
                currUnit.updateStatus();
            end
        end % updateStatusAll()

        % send the same cmd to all units
        function sendAll(obj, cmd)
            for i = 1:obj.unitCount
                currUnit = obj.unitList{i};
                currUnit.call(cmd);
            end
        end % sendAll()

        % turn off all modules that are still connected
        function gridOff(obj)
            currSerialList = serialportlist;
            for i = 1:obj.unitCount
                currUnit = obj.unitList{i};
                if ismember(currUnit.SP.Port, currSerialList) 
                    currUnit.call("off");
                end
            end
        end % sendAll()

        % get an unit object
        function unit = getUnit(obj, ID)
            unitIdx = find(obj.unitIDs == ID);
            unit = obj.unitList{unitIdx};
        end % getUnit()

        % set generator load
        function setGridLoad(obj)
            % return when grid has no energy consumer
            if ismember("houseload", obj.unitIDs) == 0
                return
            end
            % get house load
            unit = obj.getUnit("houseload");
            gridLoad = unit.status(4);
            gridLoad = str2double(gridLoad);
            gridLoad = gridLoad * -1;

            % set solar tracker load
            if ismember("solartracker", obj.unitIDs) == 1
                unit = obj.getUnit("solartracker");
                unitLoad = unit.status(4);
                unitLoad = str2double(unitLoad);
                unitMaxKW = unit.status(3);
                unitMaxKW = str2double(unitMaxKW);
                if (unitMaxKW >= gridLoad) && (unitLoad >= gridLoad) % unit can cover grid load
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(gridLoad));
                    gridLoad = 0;
                    % set load of other producers to 0
                    if ismember("windturbine", obj.unitIDs) == 1
                        unit = obj.getUnit("windturbine");
                        obj.writeSP(unit.SP, "setLoad");
                        obj.writeSP(unit.SP, string(0));
                    end
                    if ismember("generator", obj.unitIDs) == 1
                        unit = obj.getUnit("generator");
                        obj.writeSP(unit.SP, "setLoad");
                        obj.writeSP(unit.SP, string(0));
                    end
                    return;
                elseif (unitMaxKW < gridLoad) && (unitLoad >= gridLoad)
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(unitMaxKW));
                    gridLoad = gridLoad - unitMaxKW;
                else
                    unitLoad = min(unitLoad, unitMaxKW);
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(unitLoad));
                    gridLoad = gridLoad - unitLoad;
                end
            end % set solar load
            
            

            % set wind turbine load
            if ismember("windturbine", obj.unitIDs) == 1
               
                unit = obj.getUnit("windturbine");
                unitLoad = unit.status(4);
                unitLoad = str2double(unitLoad);
                unitMaxKW = unit.status(3);
                unitMaxKW = str2double(unitMaxKW);
                if (unitMaxKW >= gridLoad) && (unitLoad >= gridLoad) % unit can cover grid load
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(gridLoad));
                    gridLoad = 0;
                    % set load of other producers to 0
                    if ismember("generator", obj.unitIDs) == 1
                        unit = obj.getUnit("generator");
                        obj.writeSP(unit.SP, "setLoad");
                        obj.writeSP(unit.SP, string(0));
                    end
                    return;
                elseif (unitMaxKW < gridLoad) && (unitLoad >= gridLoad)
                    windMaxKW = unitMaxKW;
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(unitMaxKW));
                    gridLoad = gridLoad - unitMaxKW;
                else
                    unitLoad = min(unitLoad, unitMaxKW);
                    obj.writeSP(unit.SP, "setLoad");
                    obj.writeSP(unit.SP, string(unitLoad));
                    gridLoad = gridLoad - unitLoad;
                end
            end

            % set power plant load
            if ismember("generator", obj.unitIDs) == 1
          
                unit = obj.getUnit("generator");
                obj.writeSP(unit.SP, "setLoad");
                obj.writeSP(unit.SP, string(gridLoad));
                gridLoad = 0;
            end
        end

    end
end

