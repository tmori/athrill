package application;


import Code.Code;
import javafx.fxml.FXML;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextArea;

public class SampleController {
	@FXML
	private Tab fileName;
	
	@FXML
	private TextArea lineArea;
	@FXML
	private TextArea codeArea;
	
	@FXML
	private TextArea cpu0;
	@FXML
	private TextArea cpu1;
	
	@FXML
	private TabPane Tab;
	
	public TabPane getTabPane() {
		return Tab;
	}
	
	private Code code;
	
	public SampleController() {
	}
	
	public void setCode(Code code) {
		this.code = code;
	}

	public TextArea getLineArea() {
		return lineArea;
	}

	public TextArea getCodeArea() {
		return codeArea;
	}
	
	

	public Tab getFileName() {
		return fileName;
	}

	public TextArea getCpu0() {
		return cpu0;
	}

	public TextArea getCpu1() {
		return cpu1;
	}

	public Code getCurrentCode() {
		return code;
	}
	private int currentLineno;
	public void setLineno(int lineno) {
		currentLineno = lineno;
	}
	public int getCurrentLineno() {
		return currentLineno;
	}
	
}
