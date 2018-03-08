package Code;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.sun.javafx.collections.MappingChange.Map;

public class Code {
	private List<CodeFragment> lines = new ArrayList<>();
	private File file;
	private int currentCodePos = 0;
	private int currentLinePos = 0;
	private static HashMap<String, Code> map = new HashMap<String, Code>();
	
	public Code(String filepath) {
		this.file = new File(filepath);
	}
	
	public static Code getCode(String filepath) throws FileNotFoundException, IOException {
		if (!map.containsKey(filepath)) {
			Code code = new Code(filepath);
			code.build();
			map.put(filepath, code);
		}
		return map.get(filepath);
	}
	
	private void build() throws FileNotFoundException, IOException {
	    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
	        String string = reader.readLine();
	        while (string != null){
	        	this.append(string);
	            string = reader.readLine();
	        }
	    }		
	}
	
	private void append(String codeFragmentLine) {
		CodeFragment line = new CodeFragment(currentCodePos, currentLinePos, this.lines.size() + 1, codeFragmentLine);
		this.lines.add(line);
		this.currentCodePos += line.getCodeFragment().length();
		this.currentLinePos += line.getLineno().length();
	}
	
	public CodeFragment getLine(int lineno) {
		return lines.get(lineno - 1);
	}
	public CodeFragment getFromCodePos(int selectStartPos) {
		return this.lines.stream()
			.filter(line -> (line.getCodePos() <= selectStartPos) && (selectStartPos < line.getCodePosEnd()))
			.findFirst()
			.orElse(null);
	}

	public List<CodeFragment> getLines() {
		return lines;
	}

	public String getFilename() {
		return file.getName();
	}
	
}
