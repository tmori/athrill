package application;
	

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import Code.Code;
import Code.CodeFragment;
import Code.CpuInfoFile;
import Code.DebugFile;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.concurrent.ScheduledService;
import javafx.concurrent.Task;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.control.SingleSelectionModel;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.fxml.FXMLLoader;



public class Main extends Application {
	private SampleController root;
	private List<SampleController> ctrls = new ArrayList<>();
	
	private void updateLineFocus(SampleController localCtrl, int lineno) throws IOException {
		localCtrl.setLineno(lineno);
		CodeFragment line = localCtrl.getCurrentCode().getLine(lineno);
		localCtrl.getCodeArea().selectRange(line.getCodePos(), line.getCodePos() + line.getCodeFragment().length());
		localCtrl.getLineArea().selectRange(line.getLinePos(), line.getLinePos() + line.getLineno().length());
		SingleSelectionModel<Tab> selectionModel = root.getTabPane().getSelectionModel();
		selectionModel.select(localCtrl.getFileName());

	}
		
	public void setNewCode(SampleController localCtrl, DebugFile dbgFile) throws FileNotFoundException, IOException {
		int lineno = dbgFile.getLineno();
		Code currentCode = Code.getCode("/home/tmori/project/sample/os/atk2-sc1-mc_1.4.2/OBJ/" + dbgFile.getFilePath());
		//System.out.println("change Start currentCode=" + dbgFile.getFileName());
		localCtrl.setCode(currentCode);
		localCtrl.getCodeArea().clear();
		localCtrl.getLineArea().clear();
		currentCode.getLines().stream()
			.forEach( line -> {
				localCtrl.getCodeArea().insertText(line.getCodePos(), line.getCodeFragment());
				localCtrl.getLineArea().insertText(line.getLinePos(),  line.getLineno());
			});
		//System.out.println("change End currentCode=" + dbgFile.getFileName());
		updateLineFocus(localCtrl, lineno);
	}
	
	public void update() throws FileNotFoundException, IOException {
		//updateCpuInfo();
		DebugFile dbgFile = new DebugFile("/home/tmori/project/sample/os/atk2-sc1-mc_1.4.2/OBJ/arg_sakura.txt");
		int lineno = dbgFile.getLineno();
		String fileName = dbgFile.getFileName();
		SampleController localCtrl = null;
		
		if (ctrls.isEmpty()) {
			ctrls.add(root);
			root.getFileName().setText(fileName);
			setNewCode(root, dbgFile);
			return;
		}
		
		for (SampleController entry : ctrls) {
			if (entry.getFileName().getText().equals(fileName)) {
				localCtrl = entry;
				break;
			}
		}

		if (localCtrl != null) {
			updateLineFocus(localCtrl, lineno);
		}
		else {
			FXMLLoader loader = new FXMLLoader(getClass().getResource("Sample.fxml"));
			loader.load();
			SampleController newCtrl = loader.getController();
			root.getTabPane().getTabs().add(newCtrl.getFileName());
			ctrls.add(newCtrl);
			newCtrl.getFileName().setText(fileName);
			newCtrl.getLineArea().scrollTopProperty().bindBidirectional(newCtrl.getCodeArea().scrollTopProperty());
			newCtrl.getCodeArea().setFocusTraversable(true);

			setNewCode(newCtrl, dbgFile);
		}

		
	}
	
	
	@Override
	public void start(Stage primaryStage) {
		try {
			FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("Sample.fxml"));
			fxmlLoader.load();
			Scene scene = new Scene(fxmlLoader.getRoot(),700,700);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
			primaryStage.setScene(scene);
			
			primaryStage.setTitle("Athrill Debugger");
			root = fxmlLoader.getController();
			this.update();
			primaryStage.show();
			root.getLineArea().scrollTopProperty().bindBidirectional(ctrls.get(0).getCodeArea().scrollTopProperty());
			root.getCodeArea().setFocusTraversable(true);

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
