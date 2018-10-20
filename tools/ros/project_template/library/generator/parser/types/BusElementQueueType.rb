class BusElementQueueType
    @@objs = Array.new()
    attr_accessor :name
    attr_accessor :dataElmType
    attr_accessor :len
    attr_accessor :attribute

    def initialize(name, dataElmTypeRef, len, attribute)
        self.name = name
        self.len = len
        self.attribute = attribute

        elmName = dataElmTypeRef.split("/")[0]
        type = dataElmTypeRef.split("/")[1]
        if elmName == "DataElementArrayType"
            self.dataElmType = DataElementArrayType.get(type)
        else
            self.dataElmType = DataElementSingleType.get(type)
        end

        BusElementQueueType.register(self)

    end

    def self.add(name, dataElmTypeRef, len, overWrite)
        BusElementQueueType.new(name, dataElmTypeRef, len, overWrite)
    end

    def self.register(obj)
        @@objs.push(obj)
    end

    def self.get(name)
        for elm in @@objs do
            if elm.name == name then
                return elm
            end
        end
        p "ERROR: BusElementQueueType(" + name + ") is not found!"
        return nil
    end
end