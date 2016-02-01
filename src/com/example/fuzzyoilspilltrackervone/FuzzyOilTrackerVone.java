package com.example.fuzzyoilspilltrackervone;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

import com.fuzzylite.Engine;
import com.fuzzylite.Op;
import com.fuzzylite.rule.Rule;
import com.fuzzylite.rule.RuleBlock;
import com.fuzzylite.term.SShape;
import com.fuzzylite.term.Triangle;
import com.fuzzylite.term.ZShape;
import com.fuzzylite.variable.InputVariable;
import com.fuzzylite.variable.OutputVariable;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.SweepGradient;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.StrictMode;
import android.text.Editable;
import android.text.TextWatcher;
import android.text.format.Formatter;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.ToggleButton;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.TwiMaster;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;




public class FuzzyOilTrackerVone extends IOIOActivity implements SensorEventListener{
	
	
	public double BxOld = 0;
	private TextView Mx,My,Dx,Dy,IP,Beta,Dist,Theta,Mode,Ite,Motors,Con,TxtVx,TxtVy,TxtVt;
	private EditText Port, LYID;
	private ImageView Img,Circle;
	private Button Exit,Reset;
	private ToggleButton TB, TBL;
	private LinearLayout LY;
	int RB = 0, LB=0;
	private String IPtxt;
	private long timestamp;
	int thetaDot;
	private double theta, mCurrentDegree,omegaMagnitude;
	private long initTime;
	private boolean UDPrunning = true;
	private int d,d0, bta;
	private SensorManager mSensor;
	private Sensor Gyr;
	protected byte[] recievedData = new byte[34];
	protected DatagramPacket recv_packet=new DatagramPacket(recievedData, recievedData.length);
	protected ReadUDP readUDP = new ReadUDP();
	public DigitalOutput led_;
	public TwiMaster twi;
	public int port;
	public byte[] request={127,127};
	private boolean init = true;
	public int state = 1;
	private double gamma;
	private long t;
	private long oldT;
	private double oldTheta;
	public double dT;
	private int Rm;
	private int Lm;
	public double ByOld;
	private int MC= 0; // Constant Motor Speed
	private int MCL= 45;
	private double dEdt,iEdt,dist,err,dTheta,oldError,dErr,LBS,RBS;
	private double KpS,KiS,KdS,KpR,KiR,KdR;
	private int Bx,By, Ax,Ay;
	private int id=0;
	private TextView Degree;
	private final double  NS2S = (double) (1e-9f);
	private double IET,DET,V,ETold;
//	private double gammaDot;
	private double Go;
	private double Turn;
	public long NewTime;
	private int TurnPlus;
	private int TurnMines;
	private String ct;
	public double[] X={0,0,0,0};
	public double[] Y={0,0,0,0};
	private double robotSaftyRadius = 30;
	public double Lthr=65;
	public double Kl;
	private double Vm;
	private double thetaShow;
	private int agentNumber;
	
	private int SwCx;
	private int SwCy;
	protected float mCurrentBta;
	private float mLightQuantity;
	private Sensor mLightSensor;
	private long ThreadToc;
	
	final int StraightWideGate = 45;
	final int ForceMargin = 5;
	public int L;
	public int realTimeBy,realTimeBx;

	private long ThreadTic;;
	private int delay = 0;
	protected int ded = 30;
	double Fx = 0,Fy = 0, distance_sf;
	double distance_ ;

	private double Cdt;
	private double[] F={0,0};
	public double fx;
	public double fy;
	public double factorF;
	private double Vt,Vy,Vx;
	TacoMeter taco = new TacoMeter();

	private boolean swappedAx = false;

	private InputVariable light = new InputVariable();  // Interagent distance fuzzy 
	private OutputVariable Direc = new OutputVariable();// Interagent balance fuzzy  
	RuleBlock ruleBlock = new RuleBlock();
	Engine engine = new Engine();
	private double IAD_;
	private double fuzzyOldOutput;
	private boolean isRobotRepulsed;
	
	
	
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_fuzzy_oil_tracker_vone);
		
		StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
	    StrictMode.setThreadPolicy(policy);
	    
	   /* try{
	    	File file = new File("Ex.Set.txt");
    		StringBuilder text = new StringBuilder();
    		BufferedReader br = new BufferedReader(new FileReader(file));
    	    String line = null;
    	        text.append(line);
    	        br.close();
	    }catch(Exception e){
	    	
	    	nop();
	    }*/
	    
	    taco.start();
		LY = (LinearLayout)findViewById(R.id.LY);
		LY.setBackgroundColor(Color.parseColor("#FF0000"));
		LYID = (EditText)findViewById(R.id.EdtLY);
		LYID.addTextChangedListener(new TextWatcher() {
			
			@Override
			public void onTextChanged(CharSequence s, int start, int before, int count) {}
			
			@Override
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
			
			@Override
			public void afterTextChanged(Editable s) {
				// TODO Auto-generated method stub
				try{
					id = Integer.parseInt(LYID.getText().toString()) -1;
				switch(id){
				case(0):LY.setBackgroundColor(Color.parseColor("#FF0000"));break;
				case(1):LY.setBackgroundColor(Color.parseColor("#00FF00"));break;
				case(2):LY.setBackgroundColor(Color.parseColor("#0000FF"));break;
				case(3):LY.setBackgroundColor(Color.parseColor("#FFFFFF"));break;
				}
				}catch(Exception e){}
				
			}
		});
		Img = (ImageView)findViewById(R.id.IMG);
		Circle = (ImageView)findViewById(R.id.circle);
		Mx		=(TextView)findViewById(R.id.TxtMx);
		My		=(TextView)findViewById(R.id.TxtMy);
		
//		TxtVx		=(TextView)findViewById(R.id.TxtVx);
//		TxtVy		=(TextView)findViewById(R.id.TxtVy);
//		TxtVt		=(TextView)findViewById(R.id.TxtV);

		Con 	=(TextView)findViewById(R.id.textCon);
		Mode	=(TextView)findViewById(R.id.txtdistance);
		
		Dx		=(TextView)findViewById(R.id.TxtDx);
		Dy		=(TextView)findViewById(R.id.TxtDy);
		
		IP		=(TextView)findViewById(R.id.TxtIP);

		Ite 	=(TextView)findViewById(R.id.TxtTimeIte);
		Motors 	=(TextView)findViewById(R.id.Motots);
		
		Degree = (TextView)findViewById(R.id.txterror);
		Beta	=(TextView)findViewById(R.id.TxtBeta);
		Theta	=(TextView)findViewById(R.id.TxtError);
		Dist	=(TextView)findViewById(R.id.TxtDist);
		Port	= (EditText)findViewById(R.id.ETXPort);
		Port.addTextChangedListener(new TextWatcher() {
			
			@Override
			public void onTextChanged(CharSequence s, int start, int before, int count) {}
			
			@Override
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
			
			@Override
			public void afterTextChanged(Editable s) {port = Integer.parseInt(Port.getText().toString());}		});
		
		Exit	=(Button)findViewById(R.id.Bexit);
		Exit.setOnClickListener(exirListener);
		
		Reset	=(Button)findViewById(R.id.Brst);
		Reset.setOnClickListener(resetListener);

		TB 		=(ToggleButton)findViewById(R.id.TB);
		TB.setOnClickListener(tbListener);
		
		TBL		=(ToggleButton)findViewById(R.id.TBLight);
		
		mSensor = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		Gyr = mSensor.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
 		mLightSensor = mSensor.getDefaultSensor(Sensor.TYPE_LIGHT);
//		Acc = mSensor.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		initTime = System.currentTimeMillis();
		
		port = Integer.parseInt(Port.getText().toString());
		
		goReadIP();
 		refereshTheScreen();
 		doCalculation(false); readUDP.execute();
// 		FindNewDirection mythread = new FindNewDirection();
// 		mythread.start();
 		
	}
	
	private void nop() {
		// TODO Auto-generated method stub
		
	}

	SensorEventListener LightVisor = new SensorEventListener() {
		
		@Override
		public void onSensorChanged(SensorEvent event) {
			// TODO Auto-generated method stub
//		            mLightQuantity =  (float) (mLightQuantity + 0.02*(event.values[0]-mLightQuantity)/(0.02+0.15));
			 mLightQuantity = (float) (0.90*mLightQuantity+0.10*event.values[0]);
//			 L = (int) mLightQuantity;//(35*(mLightQuantity)/Lthr);
		}
		
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
			// TODO Auto-generated method stub
			
		}
	};
	OnClickListener exirListener=new OnClickListener() {
		@Override
		public void onClick(View v) {try{request[0]=127;request[1]=127;	motorShift(request);}catch(Exception e){}	
		
	/*	try {
	        OutputStream out = openFileOutput("Ex.set.txt", MODE_PRIVATE);
			OutputStreamWriter outputStreamWriter = new OutputStreamWriter(out);
	        outputStreamWriter.write(String.valueOf(id));
	        outputStreamWriter.close();
	    	
		}catch(Exception e){}*/
		
		
		System.exit(0);	}};
	
	OnClickListener resetListener = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			// TODO Auto-generated method stub
			RB = 0;LB = 0; err = 0; d = 0; d0 =0; 
			Ax = 0;Bx = 0; Ay  = 0;By = 0; init = true;
			dErr = 0;IET = 0;iEdt = 0;
			refereshTheScreen();}		};
	
	
	OnClickListener	tbListener =new OnClickListener() {
				
				@Override
				public void onClick(View v) {
//					// TODO Auto-generated method stub
//					state = 1;
//					try{
//						request[0]=127;request[1]=127;
//						motorShift(request);
//					}catch(Exception e){}
					}};
			

		private void doCalculation(boolean SelfCall) {
		// TODO Auto-generated method stub
		try{
			HighLevelControl();
			if((Bx!=realTimeBx)|(By!=realTimeBy)){
				IET = 0;iEdt = 0;
//			Bx = (int) (0.8*Bx+0.2*realTimeBx);
//			By = (int) (0.8*By+0.2*realTimeBy);
				
				Bx = (int) (realTimeBx);
				By = (int) (realTimeBy);
			}
			
//			Bx = Ax + 20;
//			By = Ay ;
			
			if(Bx>Ax){
				bta =(int) (180*(Math.atan( ((double)(By-Ay)/(Bx-Ax)))/Math.PI));
			}else if(Ax==Bx){if(By>Ay){bta = 90;}else if(By<Ay){bta = -90;}else{bta = 0;}
			}else{
				if(Ay==By){bta = 180;
				}else{bta =(int) (180+180*(Math.atan((double) (By-Ay)/(Bx-Ax))/Math.PI));}
			}
			
			bta = normAngle(bta);
			
			if((bta - d)>180){theta = (bta -d)-360;
					}else if((bta - d)<-180){theta = 360+(bta -d);
					}else{theta = bta - d;}
			if(Ax==-127 & Ay==-127){
				V =0;
			}else{
		V = Math.sqrt((By-Ay)*(By-Ay)+(Bx-Ax)*(Bx-Ax));}
			
			///													BIdirectional Motion
			if(!SelfCall) {
				if (((theta>90)|(theta<=-90))){
				d0 = normAngle(d0-180);
				bta = normAngle(bta+180);
				theta = normAngle(bta -d);
				swappedAx = true;}else{swappedAx = false;}
//				doCalculation(true);
			}
			/////
			
			
		}catch(Exception e){}

		if(d0==0){
			thetaShow = theta;
		}else{thetaShow = normAngle((int) (theta +180));}
	}
	

	private double saturate(double distance_sf, int i) {
		// TODO Auto-generated method stub
		if(distance_sf>i){
			distance_sf = i;
		}
		return distance_sf;
	}


	private int normAngle(int i) {
		// TODO Auto-generated method stub
		while(i>180){
			i = i -360;
		}
		while(i<=-180){
			i=i +360;
		}
		return i;
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
		mSensor.registerListener(this, Gyr, SensorManager.SENSOR_DELAY_FASTEST);
		mSensor.registerListener(LightVisor, mLightSensor, SensorManager.SENSOR_DELAY_FASTEST);
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
		mSensor.unregisterListener(this, Gyr);
		mSensor.unregisterListener(this, mLightSensor);
//		wakeLock_.release();
		
		try{
			motorShift(null);
		}catch(Exception e){}
		
	}
	
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (connectionType.equals("ioio.lib.android.accessory.AccessoryConnectionBootstrap.Connection")
				|| connectionType.equals("ioio.lib.impl.SocketIOIOConnection")){ct ="USB";}else if(connectionType
						.equals("ioio.lib.android.bluetooth.BluetoothIOIOConnection")){ct = "BT";}
			return new Looper();
		}
	
	
	
	public synchronized void motorShift(byte[] request){
		if (request==null){
			request =  new byte[] {127,127};
		}

	byte[] response = new byte[1] ;
	try{
		if(!TB.isChecked()){
			request[0] = 127;
			request[1] = 127;
		}
			twi.writeRead(4,false, request, request.length, response, 1);
			Thread.sleep(10);
	}catch(InterruptedException e){

	} catch (ConnectionLostException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	}
	
	class Looper extends BaseIOIOLooper{


		

		@Override
		protected void setup() throws ConnectionLostException, InterruptedException {
			// TODO Auto-generated method stub
			super.setup();
			led_ = ioio_.openDigitalOutput(0, false);
			twi = ioio_.openTwiMaster(1, TwiMaster.Rate.RATE_100KHz, false);
			KpS = 2; KdS =0; KiS = 0;
			
			KpR = 2; KdR =00; KiR = 00;

		    formFuzzy();
//			motorTest();
		}
		
		@Override
		public void loop() throws ConnectionLostException, InterruptedException {
			// TODO Auto-generated method stub
			super.loop();
			port   = Integer.parseInt(Port.getText().toString());
			led_.write(!TB.isChecked());
	 		doCalculation(false); 

	 		refereshTheScreen();
	 		
	 		if(true){
	 			
	 			
	 			switch(state){
	 			case 1:{
	 				modePause();break;}
	 			case 2:{
	 				modeRotate();break;}
	 			case 3:{
	 				modeStraight();break;}
	 			}
	 			if(BxOld!=Bx | ByOld!=By){
	 				if(V >ForceMargin){
	 					if (Math.abs(theta)>StraightWideGate){
	 						  state = 2;
	 					}else{state = 3;}
	 					}else{state = 1;}}; 	// Destination Change sensed!
	 			BxOld = Bx; ByOld = By;
	 		}
	 		}
		}
	

	private void goReadIP() {
		// TODO Auto-generated method stub

		getSystemService(Context.CONNECTIVITY_SERVICE);
		WifiManager wm = (WifiManager)getSystemService(WIFI_SERVICE);
		IPtxt = Formatter.formatIpAddress(wm.getConnectionInfo().getIpAddress());
		IP.setText(IPtxt);
	}

	public void motorTest() throws InterruptedException {
		// TODO Auto-generated method stub
		
		TBtoggle();
			request[0]=(byte) 127;
			request[1]=(byte) 127;
		while(thetaDot==0){
			TurnPlus = TurnPlus +1;
			request[0] =(byte) (request[0] +1);
			request[1] =(byte) (request[1] -1) ;
			motorShift(request);
			Thread.sleep(200);
		}
		request[0] = 127;
		request[1] = 127;
		motorShift(request);
		Thread.sleep(500);
		while(thetaDot==0){
			TurnMines = TurnMines +1;
			request[0] =(byte) (request[0] -1);
			request[1] =(byte) (request[1] +1) ;
			motorShift(request);
			Thread.sleep(200);
		}

		request[0] = 127;
		request[1] = 127;
		motorShift(request);
		Thread.sleep(500);
		
		TBtoggle();
	}

	private void TBtoggle() {
		// TODO Auto-generated method stub
		runOnUiThread(new Runnable() {
			
			@Override
			public void run() {
				// TODO Auto-generated method stub
				TB.toggle();
			}
		});
	}


	public void modeStraight() {
		// TODO Auto-generated method stub
		IET = 0;
		iEdt = 0;
		boolean notInitLoop = false;
while(state==3){
	if ((Ax==-127)&(Ay == -127)){state =1;}   		// Pause
	if ((KpS==0)&(KiS == 0)&(KdS==0)){state =1;}  	// Pause
	doCalculation(false);
	if(theta>15){theta = 15;}
	if(theta<-15){theta = -15;}
//	state = 1;
		t = System.nanoTime();
		dT = (t - oldT)*1e-9;
		dTheta = (theta - oldTheta)/((double) dT);
//		Vm = (Ax-oldX)*Math.cos(d)/(dT) + (Ay - oldY)*Math.sin(d)/(dT);
		err = V;
	if(err>25){err = 25;}
	if(err<-25){err= -25;}
		dErr = err - oldError;
		dEdt = dErr/((double) dT);
//		iErr = iErr + err;
		if(notInitLoop){
		IET = IET 	+ theta *((double) dT);
		iEdt = iEdt + err * ((double)dT);
		}else{IET = 0;notInitLoop=true;}
		

		if(IET>250){IET=0;}
		if(IET<-250){IET=-0;}
		if(iEdt>250){iEdt=0;}
		if(iEdt<-250){iEdt=-0;}

		Go = KpS*err + KdS*dEdt + KiS*iEdt;
		if (Math.abs(Go)>MCL){Go = MCL;}
		
		Turn = 	KpR*theta+ KdR *dTheta + KiR*IET;
		if (Math.abs(Turn)> 20){Turn = Math.signum(Turn)*20;}
		
		if(d0!=0){

			RBS = -Go + Turn;
			LBS = -Go - Turn;
		}else{
			RBS = Go + Turn;
			LBS = Go - Turn;
		}
		Rm = (int) (127+ MC +RBS);
		Lm = (int) (127+ MC +LBS);
//		if(Math.abs(127-Rm)>MCL){Rm = 127 + (int) Math.signum(Rm-127)*MCL;}
//		if(Math.abs(127-Lm)>MCL){Lm = 127 + (int) Math.signum(Lm-127)*MCL;}
		request[0] = (byte) Rm;
		request[1] = (byte) Lm;
		motorShift(request);
//		long delay = 100000000 - (System.nanoTime()-t);
		try {
			if (ct.equals("USB")){
//			Thread.sleep((long) (delay*1e-6));}else{Thread.sleep((long) (delay*1e-6));}
					Thread.sleep(180);}else{Thread.sleep(20);}
				} catch (InterruptedException e) {
		}
	
		
		if(V<ForceMargin){ state=1; // 			Terminate straightMode() and wait for new destination
		request[0]=(byte) (127);request[1]=(byte) (127);
//		request[0]=(byte) (127-RBS/2);request[1]=(byte) (127-LBS/2);
		motorShift(request);
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {	e.printStackTrace();}
		}
		
		
		oldT = t;
	oldTheta = theta;
	oldError = err;
			}
refereshTheScreen();


	}

	public void modeRotate() {
		// TODO Auto-generated method stub
		IET = 0;
		iEdt = 0;
		state = 3;
		boolean notInitLoop = false;
		while(state==2){
			if ((Ax==-127)&(Ay == -127))		{state = 1;}
			if ((KpR==0)&(KiR == 0)&(KdR==0))	{state = 1;}
			if (V<ForceMargin)					{state = 1;}
//			state = 3;
			doCalculation(false);
			if(theta>30){theta = 30;}else if(theta<-30){theta = -30;} 
			if(Math.abs(theta)<StraightWideGate){state =3; break;}else { // Rotation Terminate Condition
			t = System.nanoTime();
			dT = (int) (t - oldT)*1e-9;
//			ET  = theta - PIDT;
			if(notInitLoop){IET = IET + theta * dT;}else{IET = 0;notInitLoop = true;}
			if(IET>250){IET=0;}else	if(IET<-250){IET=0;}
			DET = (theta - ETold)/(dT);

			Turn = KpR*theta+KdR *DET+KiR*IET;
			if((thetaDot==0)&(Turn>0)&(theta<TurnPlus)){Turn = Turn +TurnPlus;};
			if((thetaDot==0)&(Turn<0)&(Math.abs(theta)<TurnMines)){Turn = Turn -TurnMines;};
			if (Math.abs(Turn)> MCL){Turn = Math.signum(Turn)*MCL;}
						
				RBS = Turn;
				LBS = -Turn;
				Rm = (int) (127+ MC +RBS);
				Lm = (int) (127+ MC +LBS);
				
			}

			request[0] = (byte) Rm;
			request[1] = (byte) Lm;
			motorShift(request);
			try {
				if (ct.equals("USB")){
					Thread.sleep(180);}else{Thread.sleep(20);}
			} catch (InterruptedException e) {}
		ETold = theta;
		oldT = t;
		}
	refereshTheScreen();

	}

	public void modePause() {
	// TODO Auto-generated method stub
		IET = 0;
		iEdt = 0;
		request[0]=127;		request[1]=127;
	motorShift(request);
	doCalculation(false);
	if(V >ForceMargin){
		if (Math.abs(theta)>StraightWideGate){
			state = 2;
		}else{state = 3;}}
	refereshTheScreen();
}



	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.fuzzy_oil_tracker_vone, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == R.id.action_settings) {
			return true;
		}
		return super.onOptionsItemSelected(item);
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
        {
//			double theta = 0;
			if (timestamp != 0) {
				
				double dt = (event.timestamp - timestamp) * NS2S ;
// 			Axis of the rotation sample, not normalized yet.
//	              double axisX = event.values[0];
//	              double axisY = event.values[1];
//	              double axisZ = event.values[2];

//	               Calculate the angular speed of the sample
				thetaDot = (int) Math.toDegrees(event.values[2]);
	                omegaMagnitude =  event.values[2] *dt + omegaMagnitude;
	               gamma = (double)  Math.toDegrees(omegaMagnitude)%360;


        } 
			timestamp =  event.timestamp;

//
		if (System.currentTimeMillis()-initTime<500 | init){
	
			d0  =   (int) gamma;
			init = false;
		}else{

			d = (int) (gamma - d0);
			if (d>180){d=d-360;}
			if(d<-180){d=360+d;}
		}
		}
		

//		if(d0==0){
//			thetaShow = theta;
//		}else{thetaShow = normAngle((int) (theta +180));}		
		refereshTheScreen();
		
	}
	
	private int mean(double[] x2) {
		// TODO Auto-generated method stub
		agentNumber = 0; double sum = 0;
		for(int i=0;i< x2.length;i++){
			if(x2[i]!=-127){
				sum = sum + x2[i];
				agentNumber ++;
			}
		}
		if(agentNumber==0){agentNumber=1;}
		return (int)sum/agentNumber;
	}
	
	private void refereshTheScreen() {
		// TODO Auto-generated method stub
		runOnUiThread(new Runnable() {

			@Override
			public void run() {
				try{
				Mx.setText(String.valueOf(Ax)+" ("+String.valueOf(Bx)+")");
				My.setText(String.valueOf(Ay)+" ("+String.valueOf(By)+")");
//				TxtVx.setText(String.format("%.2f",Vx));
//				TxtVy.setText(String.format("%.2f",Vy));
//				TxtVt.setText(String.format("%.2f",Vt));
				
				Dx.setText(String.format("%.2f",F[0]));
				Dy.setText(String.format("%.0f",F[1]));
				
				Degree.setText("Deg: "+String.valueOf(d));
				Beta.setText("Beta: "+ String.valueOf(bta));
				Theta.setText("Theta: "+String.valueOf((int) thetaShow));
				Dist.setText(String.format("%.2f /", V)+String.format("%.2f", Vt));
				Ite.setText(String.format("%.2f", KpR)+" "+String.format("%.2f", KdR)+" "+String.format("%.2f", KiR)+" "+String.format("%.2f", KpS)+" "+String.format("%.2f", KdS)+" "+String.format("%.2f", KiS)+" ");
				Motors.setText(String.valueOf(Rm)+", "+String.valueOf(Lm)+", "+String.valueOf(TurnPlus)+", "+String.valueOf(TurnMines)+", "+String.format("%.6f",Cdt));
				Con.setText(String.valueOf(L)+"{"+String.valueOf(Lthr)+"}"+" / "+String.valueOf(agentNumber)+":"+String.format("%.2f",V)+" /"+String.valueOf(bta));//+String.valueOf(Lthr)
				switch(state){
				case(1):{Mode.setText(ct+" Pa ");break;}
				case(2):{Mode.setText(ct+" Ro ");break;}
				case(3):{Mode.setText(ct+" St ");break;}
//				case(4):{Mode.setText(ct+" Pause   ");break;}
				}
			} catch(Exception e ){}
try{
	RotateAnimation ra = new RotateAnimation((float) mCurrentDegree,d,Animation.RELATIVE_TO_SELF, 0.5f,Animation.RELATIVE_TO_SELF,0.5f);
	RotateAnimation rc = new RotateAnimation((float) mCurrentBta, (float) -thetaShow, Animation.RELATIVE_TO_SELF, 0.5f,Animation.RELATIVE_TO_SELF,0.5f);
//	ScaleAnimation sa = new ScaleAnimation(fromS, toS, fromS, toS, Animation.RELATIVE_TO_SELF, 0.5f,Animation.RELATIVE_TO_SELF,0.5f);
    ra.setDuration(0);
    rc.setDuration(0);
//    sa.setDuration(0);
    
    ra.setFillAfter(true);
    rc.setFillAfter(true);
//    sa.setFillAfter(true);
    
    Img.startAnimation(ra);
    Circle.startAnimation(rc);
//    Circle.startAnimation(sa);
    
//    fromS = toS;
    mCurrentDegree = d;
    mCurrentBta    = (float) -thetaShow;

	} catch(Exception e ){}
			}
		});
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub
		
	}
	
	private class ReadUDP extends AsyncTask<Void, Void, Void>{

		private int ind;

		@Override
		protected Void doInBackground(Void... params) {
			// TODO Auto-generated method stub

				while(UDPrunning){
					try {
//						
				DatagramSocket clientsocket=new DatagramSocket(port);
				clientsocket.receive(recv_packet);

				syncVariables();
					refereshTheScreen();
				clientsocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			}
			return null;
		}

		@Override
		protected void onPostExecute(Void result) {
			// TODO Auto-generated method stub
			super.onPostExecute(result);
			syncVariables();
			refereshTheScreen();
		}

		private void syncVariables() {
			// TODO Auto-generated method stub
			
			ind = 8*(id);
			
			Ax = ( 0xff & recievedData[ind  ])-127 ;
			Ay = ( 0xff & recievedData[ind+1])-127 ;
//			Bx = ( 0xff & recievedData[ind+2])-127 ;
//			By = ( 0xff & recievedData[ind+3])-127 ;

			X[0] = (double)( 0xff & recievedData[0])-127;
			X[1] = (double)( 0xff & recievedData[8])-127;
			X[2] = (double)( 0xff & recievedData[16])-127;
			X[3] = (double)( 0xff & recievedData[24])-127;
			
			Y[0] = (double)( 0xff & recievedData[1])-127;
			Y[1] = (double)( 0xff & recievedData[9])-127;
			Y[2] = (double)( 0xff & recievedData[17])-127;
			Y[3] = (double)( 0xff & recievedData[25])-127;
			
			KpR = ((double)( 0xff & recievedData[ind+2]))/10 ;
			KdR = ((double)( 0xff & recievedData[ind+3]))/10 ;
			KiR = ((double)( 0xff & recievedData[ind+4]))/10 ;
			
			KpS = ((double)( 0xff & recievedData[ind+5]))/10 ;
			KdS = ((double)( 0xff & recievedData[ind+6]))/10 ;
			KiS = ((double)( 0xff & recievedData[ind+7]))/10 ;
			
			Lthr = ((double)( 0xff & recievedData[32])) ;
			Kl   = ((double)( 0xff & recievedData[33]))/100;
		}

		@Override
		protected void onCancelled() {
			// TODO Auto-generated method stub
			super.onCancelled();
		}
	}
	
	private void formFuzzy() {
		// TODO Auto-generated method stub
		engine.setName("Mohsen");

		light.setName("IAD");  							// Inter Agent Distance
		light.setEnabled(true);
		if (Lthr==0){Lthr=10;}
		light.setRange(0, 2*Lthr);
		light.addTerm(new ZShape("Dark", 0, Lthr));
		light.addTerm(new Triangle("Ok", 0.5*Lthr , Lthr, 1.5*Lthr));
		light.addTerm(new SShape("Bright", Lthr , 2*Lthr));
		engine.addInputVariable(light);
		
		Direc.setName("IAB");							// Inter Agent Balance
		Direc.setEnabled(true);
		Direc.setRange(-40, 40);
		Direc.addTerm(new SShape("gFar", 0, 40));		// Get Far
		Direc.addTerm(new Triangle("Zero", -10 , 0, 10));
		Direc.addTerm(new ZShape("gCls", -40,0));  // Get Close
		engine.addOutputVariable(Direc);

		ruleBlock.addRule(Rule.parse("if IAD is Bright	then IAB is gFar",engine));
		ruleBlock.addRule(Rule.parse("if IAD is Ok		then IAB is Zero",engine));
		ruleBlock.addRule(Rule.parse("if IAD is Dark	then IAB is gCls",engine));
		engine.addOutputVariable(Direc);

		ruleBlock.addRule(Rule.parse("if IAD is Bright	then IAB is gFar",engine));
		ruleBlock.addRule(Rule.parse("if IAD is Ok		then IAB is Zero",engine));
		ruleBlock.addRule(Rule.parse("if IAD is Dark	then IAB is gCls",engine));

		engine.addRuleBlock(ruleBlock);
		
        engine.configure("", "", "Minimum", "Maximum", "Centroid");
        }
	
	private void HighLevelControl(){

		
		ThreadTic = System.nanoTime();
		
		Cdt = (ThreadTic-ThreadToc)*1e-9;
		SwCx = mean(X);
		SwCy = mean(Y);
//		SwCx = 0;
//		SwCy = 0;
		Fx = 0;Fy = 0; distance_sf =0;distance_ = 0; isRobotRepulsed = false;
		
		
		for(int i = 0;i<4;i++){											// Swarm Agents Repulsive Forces
			if(i!=id && X[i]!=-127 && Y[i]!=-127){
				distance_ = Math.sqrt((X[i]-X[id])*(X[i]-X[id])+(Y[i]-Y[id])*(Y[i]-Y[id])); // Inter agent Distance
				 distance_sf = distance_-robotSaftyRadius;
				 if(distance_sf<10){distance_sf=10;}
				 if(distance_sf<=2*robotSaftyRadius ){
						 fx = (X[id] -X[i]);							// Direction
						 fy = (Y[id] -Y[i]);
						 factorF = (Math.sqrt(fx*fx+fy*fy))*(distance_sf*distance_sf)/5500 +0.01;
						 fx = fx/factorF;
						 fy = fy/factorF;
						 if(Math.sqrt(fx*fx+fy*fy)>ForceMargin){isRobotRepulsed = true;}
						 Fx = Fx + fx;
						 Fy = Fy + fy;
			}
			}
		}
		
		distance_sf =0;distance_ = 0;									// Swarm Center Attractive Force

		distance_ = Math.sqrt((SwCx-X[id])*(SwCx-X[id])+(SwCy-Y[id])*(SwCy-Y[id]));

		L = (int) mLightQuantity;
		 if(TBL.isChecked()){			
				fx = 0;fy = 0;				// Light Sensitivity Force
				if(L>2*Lthr){L=(int) (2*Lthr);}
				
				light.setInputValue(L);
				engine.process();
		 fx = (X[id] -SwCx);
		 fy = (Y[id] -SwCy);
		 factorF = (Math.sqrt(fx*fx+fy*fy) +0.01);
		 
		 double fuzzyOutput = Kl* Direc.getOutputValue();
		 
		 if(!Double.isNaN(fuzzyOutput)){fuzzyOldOutput =fuzzyOutput;
		 }else{fuzzyOutput =fuzzyOldOutput;} 
		 
//		 F[0]= fuzzyOutput;
		 
		 
		 if(fuzzyOldOutput>25){fuzzyOldOutput = 25;};
		 if(distance_<2*robotSaftyRadius && fuzzyOldOutput<0){fuzzyOldOutput =0;}
		 fx = (fuzzyOldOutput)*fx/factorF;
		 fy = (fuzzyOldOutput)*fy/factorF;
		 Fx = Fx + fx;
		 Fy = Fy + fy;
		 
		 
		}else{
			if(distance_>2*robotSaftyRadius && !isRobotRepulsed) {
				 if(distance_<10){distance_=10;}
				 fx = (SwCx -X[id]);
				 fy = (SwCy -Y[id]);
				 factorF = (Math.sqrt(fx*fx+fy*fy))*0.15/(saturate(distance_/60,20) +0.01);
				 fx = fx/factorF;
				 fy = fy/factorF;
				 Fx = Fx + 2*fx;
				 Fy = Fy + 2*fy;
			}
		}
		
		try {
			Thread.sleep(delay);
		} catch (InterruptedException e) {}
		
		F[0]=Math.sqrt(Fx*Fx+Fy*Fy);
			 if(Fx>=0){F[1] = Math.toDegrees(Math.atan(Fy/(Fx+0.001)));
					}else{	 F[1] = 180+Math.toDegrees(Math.atan(Fy/(Fx+0.001)));}
		
		
		realTimeBx = (int) (Ax + Fx);
		realTimeBy = (int) (Ay + Fy);

				ThreadToc =  ThreadTic;
	}
	

	class TacoMeter extends Thread{

		private int AxOld;
		private int AyOld;
//		private double dtTaco;
//		private long TimeOld;
		private int SampleTime = 1;
//		private double dtTaco;
//		private long TimeOld;
		@Override
		public void run() {
			// TODO Auto-generated method stub
			super.run();
			NewTime = System.currentTimeMillis();
			while(true){
				try {Thread.sleep(1000/SampleTime);} catch (InterruptedException e) {}
				if((Ax!=AxOld)|(Ay !=AyOld)){
//					NewTime = System.nanoTime();
//					dtTaco = (NewTime - TimeOld)*1e-9;
//					Vx = (Ax-AxOld)/(dtTaco);
//					Vy = (Ay-AyOld)/(dtTaco);
					Vx = (Ax-AxOld)*SampleTime;
					Vy = (Ay-AyOld)*SampleTime;
					Vt = Math.sqrt(Vx*Vx+Vy*Vy);
					AxOld = Ax; AyOld = Ay;
//					TimeOld = NewTime;
				}else{
					Vx = 0;Vy = 0;Vt = 0;
				}
			}
			
			
		}
		
	}
	/*class FindNewDirection extends Thread{


		@Override
		public void run() {
			// TODO Auto-generated method stub
			super.run();
	while(true){
		
		ThreadTic = System.nanoTime();
		SwCx = mean(X);
		SwCy = mean(Y);
		SwCx = 0;
		SwCy = 0;
		Fx = 0;Fy = 0; distance_sf =0;distance_ = 0;

		for(int i = 0;i<4;i++){
			if(i!=id && X[i]!=-127 && Y[i]!=-127){
				 distance_ = Math.sqrt((X[i]-X[id])*(X[i]-X[id])+(Y[i]-Y[id])*(Y[i]-Y[id]));
				 distance_sf = distance_-robotSaftyRadius;
				 if(distance_sf<10){distance_sf=10;}
				 if(distance_sf<=2*robotSaftyRadius ){
						 fx = ((X[id] -X[i]));
						 fy = ((Y[id] -Y[i]));
						 factorF = (Math.sqrt(fx*fx+fy*fy))*(distance_sf*distance_sf)/2000 +0.01;
						 fx = fx/factorF;
						 fy = fy/factorF;
						 Fx = Fx + fx;
						 Fy = Fy + fy;
			}
			}
		}
		
//		if(TB.isChecked()){
		distance_ = Math.sqrt((SwCx-X[id])*(SwCx-X[id])+(SwCy-Y[id])*(SwCy-Y[id]));
//		 distance_sf = distance_;
		 if(distance_<10){distance_=10;}
		 fx = (-(X[id] -SwCx));
		 fy = (-(Y[id] -SwCy));
		 factorF = (Math.sqrt(fx*fx+fy*fy))*0.20/(saturate(distance_/60,20) +0.01);
		 fx = fx/factorF;
		 fy = fy/factorF;
		 
		 Fx = Fx + 2*fx;
		 Fy = Fy + 2*fy;
//		}
		
		
		try {
			Thread.sleep(delay);
		} catch (InterruptedException e) {}
		
		realTimeBx = (int) (Ax + Fx);
		realTimeBy = (int) (Ay + Fy);
		 b.putInt("realTimeBx", realTimeBx);
	     b.putInt("realTimeBy", realTimeBy);

				ThreadToc = System.nanoTime() - ThreadTic;
//		By = 0;
	}
			 
			 
		}
		}*/

	
}
