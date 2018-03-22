@Grab('com.xlson.groovycsv:groovycsv:0.2')

import com.xlson.groovycsv.CsvParser

if (args.length != 1) {
    println "Usage: variable_conflict_check.groovy <file>"
    return 1;
}

csv_file = args[0];

class DataAccessInfo {
	public String variable;
	public String clock;
	public String type;
	public String core;
	public String stack;
	public String func;

	public DataAccessInfo(String var, String clock, String type, String core, String stack, String func) {
		this.variable = var.trim();
		this.clock = clock.trim();
		this.type = type.trim();
		this.core = core.trim();
		this.stack = stack.trim();
		this.func = func.trim();
	}
	
}

Map<String, List<DataAccessInfo>> map = new HashMap<>();

def csv = new File(csv_file).text
def data = new CsvParser().parse(csv, separator: ',', quoteChar: '"')
data.each {
	def info = new DataAccessInfo(it.variable, 
					it.access_clock, 
					it.type,
					it.core,
					it.stack,
					it.access_func);
	def var = map.get(it.variable);
	if (var == null) {
		var = new ArrayList<>();
		map.put(it.variable, var);
	}
	var.add(info);
}

for (String key : map.keySet()) {
	def var = map.get(key);
	def readList = var.stream()
		.filter {
			it.type.equals("READ")
		}.collect();
	def writeList = var.stream()
		.filter {
			it.type.equals("WRITE")
		}.collect();

	for (def writeInfo : writeList) {
		if (readList.isEmpty()) {
			continue;
		}
		if (readList.stream()
			.every{ writeInfo.stack.equals(it.stack) }) {
			continue;
		}
		println String.format("<%s> <write(%s):%s(%s)>", 
					key, writeInfo.func,
					writeInfo.stack, writeInfo.clock);
		readList.stream()
			.filter{ !writeInfo.stack.equals(it.stack) }
			.each {
				println String.format("  +<read(%s):%s(%s)>", 
					it.func,
					it.stack, it.clock);
			}
	}
}
