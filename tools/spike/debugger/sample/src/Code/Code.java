package Code;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Code {
	private List<CodeFragment> lines = new ArrayList<>();
	private File file;
	private int currentCodePos = 0;
	private int currentLinePos = 0;
	
	public Code(String filepath) {
		this.file = new File(filepath);
	}
	
	public void build() throws FileNotFoundException, IOException {
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
