package Code;

public class CodeFragment {
	private int codePos;
	private int linePos;
	private String lineno;
	private String codeFragment;
	
	public CodeFragment(int codePos, int linePos, int lineno, String codeFragment) {
		this.codePos = codePos;
		this.linePos = linePos;
		this.lineno = String.format("%04d\n", lineno);
		this.codeFragment = codeFragment + "\n";
	}

	public String getCodeFragment() {
		return codeFragment;
	}

	public String getLineno() {
		return String.valueOf(lineno);
	}

	public int getCodePos() {
		return codePos;
	}
	public int getCodePosEnd() {
		return codePos + codeFragment.length();
	}

	public int getLinePos() {
		return linePos;
	}
}