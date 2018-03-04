package application;


import Code.Code;
import Code.CodeFragment;
import javafx.fxml.FXML;
import javafx.scene.control.IndexRange;
import javafx.scene.control.Tab;
import javafx.scene.control.TextArea;

public class SampleController {
	@FXML
	private Tab fileName;
	
	@FXML
	private TextArea lineArea;
	@FXML
	private TextArea codeArea;
	
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
	
	@FXML
	public void codeAreaonMouseClicked() {
		//IndexRange range = this.codeArea.getSelection();
		//CodeFragment line = code.getFromCodePos(range.getStart());

		//getCodeArea().selectRange(line.getCodePos(), line.getCodePos() + line.getCodeFragment().length());
		//getLineArea().selectRange(line.getLinePos(), line.getLinePos() + line.getLineno().length());

	}

	public Tab getFileName() {
		return fileName;
	}
	
}
