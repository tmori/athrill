package application;
	

import java.io.FileNotFoundException;
import java.io.IOException;

import Code.Code;
import Code.CodeFragment;
import Code.DebugFile;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.concurrent.ScheduledService;
import javafx.concurrent.Task;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.fxml.FXMLLoader;



public class Main extends Application {
	private SampleController ctrl = null;
	private Code currentCode;
	private int currentLineno;
	
	private void updateLineFocus(int lineno) throws IOException {
		currentLineno = lineno;
		CodeFragment line = currentCode.getLine(currentLineno);
		ctrl.getCodeArea().selectRange(line.getCodePos(), line.getCodePos() + line.getCodeFragment().length());
		ctrl.getLineArea().selectRange(line.getLinePos(), line.getLinePos() + line.getLineno().length());
	}
	
	public void update() throws FileNotFoundException, IOException {
		DebugFile dbgFile = new DebugFile("/home/tmori/project/sample/os/asp3/OBJ/arg_sakura.txt");
		int lineno = dbgFile.getLineno();
		if (currentCode == null || !currentCode.getFilename().equals(dbgFile.getFileName())) {
			currentCode = new Code("/home/tmori/project/sample/os/asp3/OBJ/" + dbgFile.getFilePath());
			currentCode.build();
			
			ctrl.setCode(currentCode);
			currentCode.getLines().stream()
				.forEach( line -> {
					ctrl.getCodeArea().insertText(line.getCodePos(), line.getCodeFragment());
					ctrl.getLineArea().insertText(line.getLinePos(),  line.getLineno());
				});
				
			ctrl.getFileName().setText(currentCode.getFilename());
			updateLineFocus(lineno);
		}
		else if (lineno != currentLineno) {
			updateLineFocus(lineno);
		}
		
	}
	
	
	@Override
	public void start(Stage primaryStage) {
		try {
			FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("Sample.fxml"));
			fxmlLoader.load();
			Scene scene = new Scene(fxmlLoader.getRoot(),800,600);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
			primaryStage.setScene(scene);
			
			ctrl = fxmlLoader.getController();
			this.update();
			primaryStage.show();
			ctrl.getLineArea().scrollTopProperty().bindBidirectional(ctrl.getCodeArea().scrollTopProperty());
			ctrl.getCodeArea().setFocusTraversable(true);

			Main tmp = this;
			
			ScheduledService<Boolean> service  = new ScheduledService<Boolean>()
			{
			    @Override
			    protected Task<Boolean> createTask()
			    {
			        Task<Boolean> task = new Task<Boolean>()
			        {
			            @Override
			            protected Boolean call() throws Exception
			            {
			                Thread.sleep( 500 );
			                Platform.runLater( () -> {
								try {
									tmp.update();
								} catch (FileNotFoundException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								} catch (IOException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
							} );
			                return true;
			            };
			        };
			        return task;
			    };
			};
			 
			service.start();
			
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public static void main(String[] args) {
		launch(args);
	}
}
