@Grab('com.xlson.groovycsv:groovycsv:0.2')

import com.xlson.groovycsv.CsvParser

if (args.length != 2) {
    println "Usage: variable_conflict_check.groovy <file> <interval>"
    return 1;
}

csv_file = args[0];
int interval = Integer.parseInt(args[1]);

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
		
		DataAccessInfo readInfo = null;
		int wclock = Integer.parseInt(writeInfo.clock);
		int min = Integer.MAX_VALUE;
		readList.stream()
			.filter{ !writeInfo.stack.equals(it.stack) }
			.each {
				int tmp_clock = Integer.parseInt(it.clock);
				if (min > Math.abs(tmp_clock - wclock)) {
					min = Math.abs(tmp_clock - wclock);
					readInfo = it;
				}
			}
		
		if ((interval > 0) && Math.abs(Integer.parseInt(writeInfo.clock) - Integer.parseInt(readInfo.clock)) > interval) {
			continue;
		}

		println String.format("%d W:%s %s (%s) : %s(%s)", 
					Integer.parseInt(writeInfo.clock) - Integer.parseInt(readInfo.clock),
					key,
					writeInfo.clock, 
					writeInfo.core,
					writeInfo.stack,
					writeInfo.func);
		println String.format("  + R:%s (%s) : %s(%s)>", 
				readInfo.clock,
				readInfo.core,
				readInfo.stack, 
				readInfo.func);
	}
}
