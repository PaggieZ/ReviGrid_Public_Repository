classdef cmd < handle

    properties
        name
        unit % name of unit
        in % number of arguments MATLAB needs to read
        out % number of argumetns MATLAB needs to send
        prompts % prompts for user input
        maxRange % user input max range
        minRange % user input min range
        description % cmd description
    end

    methods
        function obj = cmd(name, inputCount, outputCount) % constructor
            obj.name = name;
            obj.in = inputCount;
            obj.out = outputCount;
        end % constructor

        function setPrompt(obj, prompts)
            obj.prompts = prompts;
        end

        function setRange(obj, min, max)
            obj.maxRange = max;
            obj.minRange = min;
        end

        function setDescription(obj, description)
            obj.description = description;
        end

        function setUnitName(obj, unitName)
            obj.unit = unitName;
        end

    end % methods
end % class