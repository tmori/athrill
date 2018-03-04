package Code;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class DebugFile {
	private File file;
	
	public DebugFile(String argFilePath) {
		this.file = new File(argFilePath);
	}
	
	public int getLineno() throws IOException {
	    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
	        String string = reader.readLine();
	        return Integer.parseInt(string.split(" ")[1]);
	    }		
	}
	
	public String getFilePath() throws IOException {
	    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
	        String string = reader.readLine();
	        return string.split(" ")[2];
	    }
	}
	public String getFileName() throws IOException {
	    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
	        String string = reader.readLine();
	        return (new File(string.split(" ")[2])).getName();
	    }
	}
}
