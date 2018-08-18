require 'yaml'
require File.dirname(__FILE__) + '/types/TypeBuilder'

config_file_folderpath = ARGV[0]
config = YAML.load_file(config_file_folderpath + "/PrimitiveType.yaml")
TypeBuilder.buildPrimitiveType(config)

config = YAML.load_file(config_file_folderpath + "/DataElementType.yaml")
TypeBuilder.buildDataElementType(config)

config = YAML.load_file(config_file_folderpath + "/BusElementType.yaml")
TypeBuilder.buildBusElementType(config)

config = YAML.load_file(config_file_folderpath + "/BusType.yaml")
TypeBuilder.buildBusType(config)

#p config

