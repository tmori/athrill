require File.dirname(__FILE__) + '/BusType'
require File.dirname(__FILE__) + '/BusElementDataType'
require File.dirname(__FILE__) + '/BusElementQueueType'
require File.dirname(__FILE__) + '/DataElementArrayType'
require File.dirname(__FILE__) + '/DataElementSingleType'
require File.dirname(__FILE__) + '/PrimitiveType'

class TypeBuilder
    def self.setBus(bus)
        @@template_bus = bus
    end
    def self.getBus()
        return @@template_bus
    end
    def self.setNodeName(nodeName)
        @@template_nodeName = nodeName
    end
    def self.getNodeName()
        return @@template_nodeName
    end

    def self.buildBusType(yaml_data)
        for bus in yaml_data[0].values[0] do
            busObj = BusType.add(bus.values[0])
            for elm in bus.values[1] do
                busObj.add(elm["name"], elm["busElmTypeRef"], elm["dir"])
            end
            #p busObj
        end
    end

    def self.buildBusElementQueueType(objs)
        for elm in objs do
            BusElementQueueType.add(elm["name"], elm["dataElmTypeRef"], elm["len"], elm["attribute"])
        end
    end
    def self.buildBusElementDataType(objs)
        for elm in objs do
            BusElementDataType.add(elm["name"], elm["dataElmTypeRef"], elm["initialValue"])
            #p BusElementDataType.get(elm["name"])
        end
    end

    def self.buildBusElementType(yaml_data)
        for elm in yaml_data do
            if elm.keys[0] == "BusElementQueueType"
                TypeBuilder.buildBusElementQueueType(elm.values[0])
            else
                TypeBuilder.buildBusElementDataType(elm.values[0])
            end
        end
    end
    def self.buildDataElementArrayType(objs)
        for elm in objs do
            DataElementArrayType.add(elm["name"], elm["pTypeRef"], elm["size"])
            #p DataElementArrayType.get(elm["name"])
        end
    end
    def self.buildDataElementSingleType(objs)
        for elm in objs do
            DataElementSingleType.add(elm["name"], elm["pTypeRef"])
            #p DataElementSingleType.get(elm["name"])
        end
    end

    def self.buildDataElementType(yaml_data)
        for elm in yaml_data do
            if elm.keys[0] == "DataElementArrayType"
                TypeBuilder.buildDataElementArrayType(elm.values[0])
            else
                TypeBuilder.buildDataElementSingleType(elm.values[0])
            end
        end
    end
    def self.buildPrimitiveType(yaml_data)
        for elm in yaml_data[0].values[0] do
            PrimitiveType.add(elm["name"], elm["size"])
            #p PrimitiveType.get(elm["name"])
        end
    end

end
